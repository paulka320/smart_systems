
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "psa/crypto.h"

#define ROLE_ENTRANCE 0
#define ROLE_ROOM1 1
#define ROLE_ROOM2 2
#define ROLE_DEMO 3
#ifndef APP_ROLE
#define APP_ROLE ROLE_DEMO
#endif


#define PIN_DOOR_IN 25
#define PIN_DOOR_OUT 26
#define PIN_RED 27
#define PIN_BUZZ 14
static const int PIR[2]   = {34, 35};
static const int LIGHT[2] = {32, 33};


#define MAX_USERS 8
#define LIGHT_WATTS 40
typedef struct { char uid[12]; char name[12]; bool inside;
                 int8_t current_room;   /* 0 = none, 1 = room1, 2 = room2 */
                 int32_t entered_at;    /* virtual seconds-of-day of room entry */
} user_t;
typedef struct { bool occupied; int64_t last_motion_us; double on_s; } room_t;


#define MAXSTAY_DEFAULT_MIN 120
typedef struct {
    char uid[12];
    int8_t room;              /* 1 or 2 */
    int32_t from_s;           /* allowed window start, seconds of day */
    int32_t to_s;             /* allowed window end, seconds of day (overnight wrap ok) */
    int32_t pass_expires;     /* 0 = none, else seconds-of-day pass expiry */
} room_perm_t;
static room_perm_t perms[MAX_USERS];
static int nperms = 0;
static int maxstay_min __attribute__((unused)) = MAXSTAY_DEFAULT_MIN;
static int maxocc[2] = { 30, 30 };            /* per-room capacity, default 30 */
static bool warned90[2] = { false, false };   /* one 90% warning per room per crossing */
static int peak_count[2] = { 0, 0 };          /* highest count since boot */
static int32_t peak_at_s[2] = { 0, 0 };       /* virtual seconds-of-day of the peak */
static bool room_auth_required = false;   /* false = default allow (demo); true once grant/revoke used */

static user_t users[MAX_USERS];
static int nusers = 0;
static room_t room[2];
static int open_min = 7 * 60, close_min = 22 * 60;   /* building access window */
static int idle_s = 20;                              /* lights off after N s without motion */
static bool evac = false, sd_ok = false;

static SemaphoreHandle_t mtx, sd_mtx;
static QueueHandle_t log_q, gate_q[2], alarm_q;

typedef struct {
    uint8_t version;
    uint8_t room_id;
    uint8_t occupied;
    uint8_t sequence;
} room_report_t;
typedef struct { bool seen, occupied; int64_t last_seen_us; } remote_room_t;
static QueueHandle_t room_report_q;
static remote_room_t remote_room[2];

/* admin session (touched only by the CLI task) */
static char pin_hash[65];
static bool admin = false;
static int64_t admin_until = 0, login_lock = 0;
static int login_fails = 0;

/* ---------- virtual clock (can be set and sped up for demos) ---------- */
static portMUX_TYPE clk_lock = portMUX_INITIALIZER_UNLOCKED;
static int base_s = 8 * 3600, speed = 1;
static int64_t base_us = 0;

static int now_s(void) {
    int b, sp; int64_t bu;
    portENTER_CRITICAL(&clk_lock); b = base_s; bu = base_us; sp = speed; portEXIT_CRITICAL(&clk_lock);
    return (int)((b + (esp_timer_get_time() - bu) * sp / 1000000) % 86400);
}
static void set_clock(int sec, int sp) {
    portENTER_CRITICAL(&clk_lock); base_s = sec; base_us = esp_timer_get_time(); speed = sp; portEXIT_CRITICAL(&clk_lock);
}
static bool in_window(int s) {            /* is building access allowed at second-of-day s? */
    int m = s / 60;
    return open_min <= close_min ? (m >= open_min && m < close_min) : (m >= open_min || m < close_min);
}

/* ---------- helpers ---------- */
/*
 * =========================================================================
 * PART B — Boot and services
 *   logev, alarm_req, ESP-NOW receive/init/report, logger_task, alarm_task,
 *   cli_task, app_main.
 * =========================================================================
 */
static void logev(const char *ev, const char *uid, const char *name, const char *detail) {
    char l[96]; int s = now_s();
    snprintf(l, sizeof l, "%02d:%02d:%02d,%s,%s,%s,%s", s / 3600, s / 60 % 60, s % 60, ev, uid, name, detail);
    printf("\n[LOG] %s\n", l);
    if (APP_ROLE == ROLE_ENTRANCE || APP_ROLE == ROLE_DEMO) xQueueSend(log_q, l, 0);
}
static void alarm_req(int ms) { xQueueSend(alarm_q, &ms, 0); }

static void room_report_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    (void)info;
    if (len != sizeof(room_report_t)) return;
    room_report_t report;
    memcpy(&report, data, sizeof(report));
    if (report.version != 1 || report.room_id < 1 || report.room_id > 2) return;
    xQueueSend(room_report_q, &report, 0);
}

static bool radio_init(bool is_gateway) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() != ESP_OK || nvs_flash_init() != ESP_OK) return false;
    } else if (err != ESP_OK) return false;

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return false;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return false;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&cfg) != ESP_OK ||
        esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK ||
        esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK ||
        esp_wifi_start() != ESP_OK || esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
        esp_now_init() != ESP_OK) return false;

    if (is_gateway && esp_now_register_recv_cb(room_report_recv_cb) != ESP_OK) return false;
    const uint8_t broadcast[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, broadcast, sizeof(broadcast));
    peer.channel = 1;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    err = esp_now_add_peer(&peer);
    return err == ESP_OK || err == ESP_ERR_ESPNOW_EXIST;
}

static void radio_report_task(void *arg) {
    (void)arg;
    room_report_t report;
    for (;;) {
        if (xQueueReceive(room_report_q, &report, portMAX_DELAY) != pdTRUE) continue;
        int idx = report.room_id - 1;
        bool changed;
        xSemaphoreTake(mtx, portMAX_DELAY);
        changed = !remote_room[idx].seen || remote_room[idx].occupied != (report.occupied != 0);
        remote_room[idx].seen = true;
        remote_room[idx].occupied = report.occupied != 0;
        remote_room[idx].last_seen_us = esp_timer_get_time();
        xSemaphoreGive(mtx);
        if (changed) {
            char name[8];
            snprintf(name, sizeof(name), "ROOM%d", report.room_id);
            logev("ROOM_UPDATE", "-", name, report.occupied ? "occupied" : "vacant");
        }
    }
}
/*
 * =========================================================================
 * PART C — Entrance access control
 *   find, count_inside, hash_pin, SD storage (sd_init, save_users, save_cfg,
 *   load_all), gate_task, scan/who/users/log commands.
 * =========================================================================
 */
static user_t *find(const char *uid) {                 /* call with mtx held */
    for (int i = 0; i < nusers; i++) if (!strcmp(users[i].uid, uid)) return &users[i];
    return NULL;
}
static int count_inside(void) {                         /* call with mtx held */
    int n = 0; for (int i = 0; i < nusers; i++) n += users[i].inside; return n;
}
static int count_in_room(int room) {                    /* call with mtx held */
    int n = 0; for (int i = 0; i < nusers; i++) if (users[i].current_room == room) n++;
    return n;
}

/* Log a single WARN when a room reaches 90% of capacity; clear the flag when it drops back. */
static void room_check_90(int room) {                   /* call with mtx held */
    int cap = maxocc[room - 1];
    if (cap <= 0) return;
    int n = count_in_room(room);
    int thr = cap * 90 / 100;
    if (n >= thr && !warned90[room - 1]) {
        warned90[room - 1] = true;
        char d[64]; snprintf(d, sizeof d, "room%d at 90%% (%d/%d)", room, n, cap);
        logev("WARN", "-", "-", d);
    }
    if (n < thr) warned90[room - 1] = false;
}

/* true if second-of-day now_s lies in [from_s, to_s); supports overnight wrap */
static bool in_time_window(int from_s, int to_s, int now_s) {
    if (from_s <= to_s) return (now_s >= from_s && now_s < to_s);
    return (now_s >= from_s || now_s < to_s);
}

/* true if the virtual clock has reached/passed a pass expiry (passes < 12 h) */
static bool pass_is_expired(int now_s, int expiry_s) {
    int f = (expiry_s - now_s + 86400) % 86400;
    return f == 0 || f > 12 * 3600;
}

typedef struct { bool allow; int8_t reason; } room_decision_t;
/* reason: 0 not in building, 1 no permission, 2 outside allowed time,
 * 3 pass expired, 4 already in room, 5 room full, 6 not in this room */
static __attribute__((unused)) room_decision_t decide_room_entry(const user_t *u, int room, bool in_op,
                                         int now_s, const room_perm_t *perms,
                                         int nperms, int maxocc, int occupants) {
    room_decision_t r = { .allow = false, .reason = -1 };
    if (in_op) {
        if (!u->inside) { r.reason = 0; return r; }                       /* not in building */
        if (room_auth_required) {
            const room_perm_t *p = NULL;
            for (int i = 0; i < nperms; i++) {
                if (strcmp(perms[i].uid, u->uid) == 0 && perms[i].room == room) { p = &perms[i]; break; }
            }
            if (!p) { r.reason = 1; return r; }                               /* no permission */
            if (!in_time_window(p->from_s, p->to_s, now_s)) { r.reason = 2; return r; } /* outside allowed time */
            if (p->pass_expires != 0 && pass_is_expired(now_s, p->pass_expires)) { r.reason = 3; return r; } /* pass expired */
        }
        if (u->current_room == room) { r.reason = 4; return r; }          /* already in room */
        if (maxocc > 0 && occupants >= maxocc) { r.reason = 5; return r; } /* room full */
        r.allow = true;
    } else {
        if (u->current_room != room) { r.reason = 6; return r; }          /* not in this room */
        r.allow = true;
    }
    return r;
}

static __attribute__((unused)) const char *room_reason_str(int8_t reason) {
    switch (reason) {
        case 0: return "not in building";
        case 1: return "no permission";
        case 2: return "outside allowed time";
        case 3: return "pass expired";
        case 4: return "already in room";
        case 5: return "room full";
        case 6: return "not in this room";
        case 7: return "no link to entrance";
        default: return "unknown";
    }
}

/* Synchronous room in/out decision used by the console (demo) and, later, the radio path.
 * Caller must hold mtx. Logs ROOM_IN / ROOM_OUT / ROOM_MOVE / ROOM_DENIED. */
static void room_scan(int room, bool in_op, const char *uid) {
    user_t *u = find(uid);
    char rname[16]; snprintf(rname, sizeof rname, "Room%d", room);
    if (!u) { logev("ROOM_DENIED", uid, "-", "unknown tag"); alarm_req(150);
              printf("Tag %s: unknown tag\n", uid); return; }
    int now = now_s();
    int old_room = u->current_room;
    int before = count_in_room(room);
    room_decision_t d = decide_room_entry(u, room, in_op, now, perms, nperms,
                                          maxocc[room - 1], before);
    if (!d.allow) {
        logev("ROOM_DENIED", uid, u->name, room_reason_str(d.reason));
        alarm_req(150);
        printf("Tag %s: %s\n", uid, room_reason_str(d.reason));
        return;
    }
    const char *nm = u->name;
    char detail[128];
    if (in_op) {
        if (old_room != 0 && old_room != room) {
            /* auto-leave previous room */
            u->current_room = 0; u->entered_at = 0;
            int after_leave = count_in_room(old_room);
            u->entered_at = now; u->current_room = (int8_t)room;
            int after_join = count_in_room(room);
            if (after_join > peak_count[room - 1]) { peak_count[room - 1] = after_join; peak_at_s[room - 1] = now; }
            snprintf(detail, sizeof detail, "Room%d count=%d -> Room%d count=%d",
                     old_room, after_leave, room, after_join);
            logev("ROOM_MOVE", uid, nm, detail);
            room_check_90(old_room);
            room_check_90(room);
        } else {
            u->entered_at = now; u->current_room = (int8_t)room;
            int after = count_in_room(room);
            if (after > peak_count[room - 1]) { peak_count[room - 1] = after; peak_at_s[room - 1] = now; }
            snprintf(detail, sizeof detail, "%s count=%d", rname, after);
            logev("ROOM_IN", uid, nm, detail);
            room_check_90(room);
        }
        printf("Tag %s presented at %s in reader\n", uid, rname);
    } else {
        u->current_room = 0; u->entered_at = 0;
        int after = count_in_room(room);
        snprintf(detail, sizeof detail, "%s count=%d", rname, after);
        logev("ROOM_OUT", uid, nm, detail);
        room_check_90(room);
        printf("Tag %s presented at %s out reader\n", uid, rname);
    }
}
static void hash_pin(const char *p, char *out) {
    unsigned char h[32]; size_t h_len = 0;
    if (psa_hash_compute(PSA_ALG_SHA_256, (const unsigned char *)p, strlen(p),
                         h, sizeof(h), &h_len) != PSA_SUCCESS || h_len != sizeof(h)) {
        out[0] = '\0';
        return;
    }
    for (int i = 0; i < 32; i++) sprintf(out + 2 * i, "%02x", h[i]);
    out[64] = '\0';
}

/* ---------- SD card storage ---------- */
static bool sd_init(void) {
    esp_vfs_fat_sdmmc_mount_config_t mc = { .format_if_mount_failed = true, .max_files = 4, .allocation_unit_size = 16 * 1024 };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus = { .mosi_io_num = 23, .miso_io_num = 19, .sclk_io_num = 18,
                             .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 4000 };
    if (spi_bus_initialize(host.slot, &bus, SDSPI_DEFAULT_DMA) != ESP_OK) return false;
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = 5; slot.host_id = host.slot;
    sdmmc_card_t *card;
    return esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot, &mc, &card) == ESP_OK;
}
static bool save_users(void) {          /* writes who-is-inside so it survives a reboot */
    if (!sd_ok) return false;
    xSemaphoreTake(sd_mtx, portMAX_DELAY);
    char b[320] = ""; int n = 0;
    xSemaphoreTake(mtx, portMAX_DELAY);
    for (int i = 0; i < nusers; i++) n += snprintf(b + n, sizeof b - n, "%s %s %d\n", users[i].uid, users[i].name, users[i].inside);
    xSemaphoreGive(mtx);
    FILE *f = fopen("/sdcard/users.csv", "w");
    bool saved = f && fputs(b, f) >= 0;
    if (f && fclose(f) != 0) saved = false;
    xSemaphoreGive(sd_mtx);
    return saved;
}
static void save_cfg(void) {
    if (!sd_ok) return;
    xSemaphoreTake(sd_mtx, portMAX_DELAY);
    FILE *f = fopen("/sdcard/config.txt", "w");
    if (f) { fprintf(f, "%d %d %d %s %d %d\n", open_min, close_min, idle_s, pin_hash, maxocc[0], maxocc[1]); fclose(f); }
    xSemaphoreGive(sd_mtx);
}
static __attribute__((unused)) bool save_rooms(void) {            /* persists permissions; lock order sd_mtx -> mtx */
    if (!sd_ok) return false;
    xSemaphoreTake(sd_mtx, portMAX_DELAY);
    xSemaphoreTake(mtx, portMAX_DELAY);
    FILE *f = fopen("/sdcard/rooms.csv", "w");
    bool saved = false;
    if (f) {
        saved = true;
        for (int i = 0; i < nperms; i++)
        fprintf(f, "%s,%d,%d,%d,%d\n", perms[i].uid, (int)perms[i].room,
                (int)perms[i].from_s, (int)perms[i].to_s, (int)perms[i].pass_expires);
        fclose(f);
    }
    xSemaphoreGive(mtx);
    xSemaphoreGive(sd_mtx);
    return saved;
}
static void load_rooms(void) {            /* called at boot; no locks needed yet */
    if (!sd_ok) return;
    FILE *f = fopen("/sdcard/rooms.csv", "r");
    if (!f) return;
    char line[128];
    while (fgets(line, sizeof line, f)) {
        char uid[12]; int room, from, to, exp;
        if (sscanf(line, "%11[^,],%d,%d,%d,%d", uid, &room, &from, &to, &exp) == 5 &&
            nperms < MAX_USERS) {
            snprintf(perms[nperms].uid, 12, "%s", uid);
            perms[nperms].room = (int8_t)room;
            perms[nperms].from_s = from;
            perms[nperms].to_s = to;
            perms[nperms].pass_expires = exp;
            nperms++;
        }
    }
    fclose(f);
}
static void add_user(const char *uid, const char *name, bool inside) {   /* mtx held or boot time */
    if (nusers >= MAX_USERS) return;
    snprintf(users[nusers].uid, 12, "%s", uid); snprintf(users[nusers].name, 12, "%s", name);
    users[nusers++].inside = inside;
}
static void load_all(void) {
    hash_pin("1234", pin_hash);                          /* default admin PIN */
    if (sd_ok) {
        FILE *f = fopen("/sdcard/config.txt", "r");
        if (f) {
            char h[65];
            if (fscanf(f, "%d %d %d %64s", &open_min, &close_min, &idle_s, h) == 4) {
                strcpy(pin_hash, h);
                int c1, c2;
                if (fscanf(f, "%d %d", &c1, &c2) == 2) { maxocc[0] = c1; maxocc[1] = c2; }
            }
            fclose(f);
        }
        f = fopen("/sdcard/users.csv", "r");
        if (f) { char u[12], nm[12]; int in; while (fscanf(f, "%11s %11s %d", u, nm, &in) == 3) add_user(u, nm, in); fclose(f); }
    }
    if (nusers == 0) { add_user("A1B2C3D4", "Alice", false); add_user("B2C3D4E5", "Bob", false); add_user("C3D4E5F6", "Carol", false); }
    load_rooms();                                             /* permissions persist across reboot */
    /* room presence is not persisted: clear it at boot and note it */
    xSemaphoreTake(mtx, portMAX_DELAY);
    for (int i = 0; i < nusers; i++) { users[i].current_room = 0; users[i].entered_at = 0; }
    xSemaphoreGive(mtx);
    logev("CONFIG", "-", "system", "room presence cleared at boot");
}

/* ---------- tasks ---------- */
static void logger_task(void *a) {                       /* the ONLY writer of access.csv */
    char l[96];
    for (;;) {
        xQueueReceive(log_q, l, portMAX_DELAY);
        if (!sd_ok) continue;
        xSemaphoreTake(sd_mtx, portMAX_DELAY);
        FILE *f = fopen("/sdcard/access.csv", "a"); if (f) { fprintf(f, "%s\n", l); fclose(f); }
        xSemaphoreGive(sd_mtx);
    }
}
static void alarm_task(void *a) {                        /* beeps + red LED, never blocks other tasks */
    int ms;
    for (;;) {
        xQueueReceive(alarm_q, &ms, portMAX_DELAY);
        gpio_set_level(PIN_RED, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128); ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(ms));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        gpio_set_level(PIN_RED, 0);
    }
}
/* Entrance terminal: one task per door section (0 = IN, 1 = OUT) */
static void gate_task(void *arg) {
    int dir = (int)(intptr_t)arg;
    const char *tag = dir ? "EXIT" : "ENTRY";
    char uid[12]; int fails = 0; int64_t first = 0, lock_until = 0;
    gpio_set_level(dir ? PIN_DOOR_OUT : PIN_DOOR_IN, 0);
    for (;;) {
        xQueueReceive(gate_q[dir], uid, portMAX_DELAY);
        int64_t t = esp_timer_get_time();
        if (t < lock_until) { logev("DENIED", uid, "-", "gate locked after repeated failures"); continue; }
        const char *why = NULL; char name[12] = "-"; bool ok = false; int inside;
        int leaving_room = 0;
        xSemaphoreTake(mtx, portMAX_DELAY);
        user_t *u = find(uid);
        if (!u) why = "unknown tag";
        else {
            snprintf(name, sizeof name, "%s", u->name);
            bool open = in_window(now_s());
            if (dir == 0) {
                if (!open) why = "building closed";
                else if (u->inside) why = "already inside (anti-passback)";
                else { u->inside = true; ok = true; }
            } else {
                if (!open && !evac) why = "exit blocked outside hours";
                else if (!u->inside) why = "not inside";
                else { u->inside = false; ok = true; }
            }
        }
        if (u && ok && dir == 1 && u->current_room != 0) {   /* leaving building while in a room */
            leaving_room = u->current_room;
            u->current_room = 0; u->entered_at = 0;
        }
        inside = count_inside();
        xSemaphoreGive(mtx);
        if (leaving_room) {
            char rn[16]; snprintf(rn, sizeof rn, "Room%d", leaving_room);
            logev("ROOM_OUT", uid, name, "auto: left building");
        }
        if (ok) {
            char d[24]; snprintf(d, sizeof d, "inside=%d", inside);
            logev(tag, uid, name, d); save_users(); fails = 0;
            gpio_set_level(dir ? PIN_DOOR_OUT : PIN_DOOR_IN, 1);   /* open door 3 s */
            alarm_req(80); vTaskDelay(pdMS_TO_TICKS(3000));
            gpio_set_level(dir ? PIN_DOOR_OUT : PIN_DOOR_IN, 0);
        } else {
            logev("DENIED", uid, name, why); alarm_req(150);
            if (!first || t - first > 30000000LL) { first = t; fails = 0; }
            if (++fails >= 3) {                                    /* tamper detection */
                logev("ALERT", uid, "-", "3 bad scans: gate locked 20s"); alarm_req(2500);
                lock_until = t + 20000000LL; fails = 0; first = 0;
            }
        }
    }
}
/* Room terminal: PIR -> light, with idle timeout, energy counter, intruder alert */
/*
 * =========================================================================
 * PART D — Rooms and radio
 *   room_task (PIR/light/heartbeat/intruder alert), show_status, motion
 *   command. ESP-NOW receive/init/report live in Part B region.
 * =========================================================================
 */
static void room_task(void *arg) {
    int r = (int)(intptr_t)arg;
    const char *nm = r == 0 ? "ROOM1" : "ROOM2";
    bool light = false, prev_occ = false, pir_armed = false;
    uint8_t sequence = 0;
    int64_t last_report_us = 0;
    int64_t last_alert_us = 0, last_info_us = 0;
    const char *rlabel = r == 0 ? "room1" : "room2";
    for (;;) {
        int64_t t = esp_timer_get_time();
        xSemaphoreTake(mtx, portMAX_DELAY);
        int pir_level = gpio_get_level(PIR[r]);
        if (!pir_level) pir_armed = true;
        else if (pir_armed) room[r].last_motion_us = t;
        bool occ = (t - room[r].last_motion_us) < (int64_t)idle_s * 1000000LL;
        int cnt = count_in_room(r + 1);
        bool motion_alert = occ && !prev_occ && cnt == 0;
        bool idle_info = cnt > 0 && (t - room[r].last_motion_us) >= (int64_t)600 * 1000000LL;
        room[r].occupied = occ; if (occ) room[r].on_s += 0.2;
        xSemaphoreGive(mtx);
        gpio_set_level(LIGHT[r], occ);
        if (occ != light) { light = occ; logev(occ ? "LIGHT_ON" : "LIGHT_OFF", "-", nm, occ ? "motion" : "room empty"); }
        if (motion_alert && (t - last_alert_us >= (int64_t)60 * 1000000LL)) {
            last_alert_us = t;
            char d[64]; snprintf(d, sizeof d, "motion in %s but nobody badged in", rlabel);
            logev("ALERT", "-", nm, d); alarm_req(3000);
        }
        if (idle_info && (t - last_info_us >= (int64_t)60 * 1000000LL)) {
            last_info_us = t;
            logev("INFO", "-", nm, "badged but room idle");
        }
        if ((APP_ROLE == ROLE_ROOM1 || APP_ROLE == ROLE_ROOM2) &&
            (occ != prev_occ || t - last_report_us >= 5000000LL)) {
            room_report_t report = { .version = 1, .room_id = (uint8_t)(r + 1),
                                     .occupied = occ ? 1 : 0, .sequence = sequence++ };
            if (esp_now_send(NULL, (const uint8_t *)&report, sizeof(report)) == ESP_OK)
                last_report_us = t;
        }
        prev_occ = occ;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
static void sched_task(void *a) {                         /* announces open/close, warns stay-behinds */
    bool prev = in_window(now_s());
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        bool o = in_window(now_s());
        if (o == prev) continue;
        prev = o;
        xSemaphoreTake(mtx, portMAX_DELAY); int n = count_inside(); xSemaphoreGive(mtx);
        char d[24]; snprintf(d, sizeof d, "inside=%d", n);
        logev(o ? "BUILDING_OPEN" : "BUILDING_CLOSED", "-", "-", d);
        if (!o && n) logev("WARN", "-", "-", "people still inside at closing");
    }
}

/* ---------- serial menu ---------- */
/*
 * =========================================================================
 * PART E — Admin security and console
 *   hash_pin, is_admin, show_help, exec, cli_task, app_main and the
 *   clearinside/login/logout/schedule/adduser/deluser/evac/passwd/settime/
 *   speed commands.
 * =========================================================================
 */
static bool is_admin(void) {
    int64_t t = esp_timer_get_time();
    if (admin && t < admin_until) { admin_until = t + 60000000LL; return true; }   /* 60 s idle timeout */
    admin = false; return false;
}
static int parse_hm(const char *s) {
    int h, m; if (sscanf(s, "%d:%d", &h, &m) != 2 || h < 0 || h > 23 || m < 0 || m > 59) return -1; return h * 60 + m;
}
static void up(char *s) { for (; *s; s++) *s = toupper((unsigned char)*s); }
static int iequals(const char *a, const char *b) {
    for (; *a && *b; a++, b++) if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) return 0;
    return *a == *b;
}
#define NEED_ADMIN if (!is_admin()) { puts("DENIED: admin only. Use: login <pin>"); return; }

static void show_rooms(void) {                              /* who is badged into each room */
    xSemaphoreTake(mtx, portMAX_DELAY);
    for (int i = 0; i < 2; i++) {
        const char *nm = i == 0 ? "Room1" : "Room2";
        int n = count_in_room(i + 1);
        bool occ = room[i].occupied;
        if (n == 0) { printf("%s: 0 inside | cap %d | peak %d | PIR: %s | light %s\n", nm, maxocc[i], peak_count[i], occ ? "motion" : "idle", occ ? "ON" : "OFF"); continue; }
        printf("%s: %d inside (", nm, n);
        int shown = 0;
        for (int j = 0; j < nusers && shown < 6; j++) {
            if (users[j].current_room != i + 1) continue;
            if (shown) printf(", ");
            printf("%s", users[j].name);
            shown++;
        }
        if (n > shown) printf(" ...and %d more", n - shown);
        printf(") | cap %d | peak %d | PIR: %s | light %s\n", maxocc[i], peak_count[i], occ ? "motion" : "idle", occ ? "ON" : "OFF");
    }
    xSemaphoreGive(mtx);
}

static void show_status(void) {
    int s = now_s(); char sp[16];
    xSemaphoreTake(mtx, portMAX_DELAY);
    double used = 0; for (int i = 0; i < 2; i++) used += room[i].on_s * LIGHT_WATTS / 3600.0;
    double base = 2.0 * LIGHT_WATTS * (esp_timer_get_time() / 1e6) / 3600.0;
    snprintf(sp, sizeof sp, "x%d", speed);
    printf("Time %02d:%02d:%02d (%s) | Building %s (%02d:%02d-%02d:%02d) | Evac %s\n", s / 3600, s / 60 % 60, s % 60, sp,
           in_window(s) ? "OPEN" : "CLOSED", open_min / 60, open_min % 60, close_min / 60, close_min % 60, evac ? "ON" : "off");
    printf("Inside (badged): %d | SD: %s | Admin: %s\n", count_inside(), sd_ok ? "OK" : "MISSING", admin ? "logged in" : "no");
    if (APP_ROLE == ROLE_ENTRANCE) {
        int64_t t = esp_timer_get_time();
        for (int i = 0; i < 2; i++) {
            const char *state = !remote_room[i].seen ? "OFFLINE" :
                (t - remote_room[i].last_seen_us > 15000000LL ? "STALE" :
                 (remote_room[i].occupied ? "OCCUPIED" : "vacant"));
            printf("Room%d: %s%s\n", i + 1, state,
                   remote_room[i].seen && t - remote_room[i].last_seen_us <= 15000000LL
                       ? (remote_room[i].occupied ? ", light ON" : ", light OFF") : "");
        }
    } else {
        for (int i = 0; i < 2; i++)
            printf("Room%d: %s, light %s\n", i + 1,
                   room[i].occupied ? "OCCUPIED" : "vacant",
                   room[i].occupied ? "ON" : "OFF");
    }
    if (APP_ROLE == ROLE_ENTRANCE || APP_ROLE == ROLE_DEMO) {
        for (int i = 0; i < 2; i++) {
            int n = count_in_room(i + 1);
            printf("Room%d: %d badged", i + 1, n);
            for (int j = 0; j < nusers; j++)
                if (users[j].current_room == i + 1) printf(" %s", users[j].name);
            puts("");
        }
    }
    printf("Energy: used %.2f Wh, saved %.2f Wh vs lights always on\n", used, base > used ? base - used : 0);
    xSemaphoreGive(mtx);
}
static void show_help(void) {
    puts("--- PUBLIC ---\n"
         " status | who | users | log [n]        view system info\n"
         " scan in|out <UID>                     present RFID tag at a door\n"
         " motion 1|2                            simulate PIR motion in a room\n"
         " count                                 building / per-room badge counts\n"
         " login <pin> | logout                  admin session (default PIN 1234)\n"
         "--- ROOM READERS ---\n"
         " scan room1|room2 in|out <UID>         present tag at a room reader\n"
         " scan room 1|2 in|out <UID>            same, with a space before the room number\n"
         " rooms                                 who is badged into each room, count, cap, peak, PIR state\n"
         "--- ADMIN ---\n"
         " schedule HH:MM HH:MM                  set building open/close time\n"
         " adduser <UID> <name> | deluser <UID>  manage tags\n"
         " clearinside                           clear saved badge-ins (building confirmed empty)\n"
         " evac on|off                           emergency: allow exit anytime\n"
         " passwd <newpin>                       change admin PIN\n"
         " settime HH:MM | speed <1-600>         demo clock tools\n"
         " maxocc <room> <n>                     room capacity (default 30)\n"
         " resetpeak                             reset per-room peak counts\n"
         " grant / revoke / pass / maxstay       room permissions (implemented next phase)");
}
static void exec(char *line) {
    char *av[8]; int ac = 0;
    for (char *p = strtok(line, " "); p && ac < 8; p = strtok(NULL, " ")) av[ac++] = p;
    if (!ac) return;
    const char *c = av[0];
    if (!strcmp(c, "help")) show_help();
    else if (!strcmp(c, "status")) show_status();
    else if (!strcmp(c, "scan")) {
        if (ac < 3) { puts("Usage: scan in|out <UID>   or   scan room1|room2 in|out <UID>"); return; }
        if (iequals(av[1], "in") || iequals(av[1], "out")) {
            if (ac != 3) { puts("Usage: scan in|out <UID>"); return; }
            up(av[2]); char uid[12]; snprintf(uid, sizeof uid, "%s", av[2]);
            xQueueSend(gate_q[!strcmp(av[1], "out")], uid, 0); printf("Tag %s presented at %s reader\n", uid, av[1]);
            return;
        }
        int room = -1, uid_index = -1; bool in_op = false;
        if (iequals(av[1], "room1")) { room = 1; uid_index = 3; }
        else if (iequals(av[1], "room2")) { room = 2; uid_index = 3; }
        else if (iequals(av[1], "room") && ac >= 5 &&
                 (iequals(av[2], "1") || iequals(av[2], "2"))) {
            room = av[2][0] - '0'; uid_index = 4;
        }
        if (room == -1 || uid_index < 0 || ac != uid_index + 1) {
            puts("Usage: scan room1|room2 in|out <UID>"); return;
        }
        if (iequals(av[uid_index - 1], "in")) in_op = true;
        else if (iequals(av[uid_index - 1], "out")) in_op = false;
        else { puts("Usage: scan room1|room2 in|out <UID>"); return; }
        up(av[uid_index]); char uid[12]; snprintf(uid, sizeof uid, "%s", av[uid_index]);
        xSemaphoreTake(mtx, portMAX_DELAY);
        room_scan(room, in_op, uid);
        xSemaphoreGive(mtx);
        return;
    }
    else if (!strcmp(c, "motion") && ac == 2 && (av[1][0] == '1' || av[1][0] == '2')) {
        xSemaphoreTake(mtx, portMAX_DELAY); room[av[1][0] - '1'].last_motion_us = esp_timer_get_time(); xSemaphoreGive(mtx);
        printf("Motion in room %s\n", av[1]);
    }
    else if (!strcmp(c, "who")) {
        xSemaphoreTake(mtx, portMAX_DELAY);
        printf("Inside now (%d):", count_inside());
        for (int i = 0; i < nusers; i++) if (users[i].inside) {
            printf(" %s", users[i].name);
            if (users[i].current_room) printf("(Room%d)", users[i].current_room);
        }
        puts(""); xSemaphoreGive(mtx);
    }
    else if (!strcmp(c, "users")) {
        xSemaphoreTake(mtx, portMAX_DELAY);
        for (int i = 0; i < nusers; i++) {
            printf("%s  %-10s %s", users[i].uid, users[i].name, users[i].inside ? "inside" : "outside");
            if (users[i].current_room) printf("   Room%d", users[i].current_room); else printf("   -");
            puts("");
        }
        xSemaphoreGive(mtx);
    }
    else if (!strcmp(c, "rooms")) { show_rooms(); }
    else if (!strcmp(c, "count")) {
        xSemaphoreTake(mtx, portMAX_DELAY);
        int inside = count_inside();
        int r1 = count_in_room(1), r2 = count_in_room(2);
        printf("Building: %d inside / %d registered | Room1: %d | Room2: %d | not in a room: %d\n",
               inside, nusers, r1, r2, inside - r1 - r2);
        xSemaphoreGive(mtx);
    }
    else if (!strcmp(c, "clearinside") && ac == 1) {
        NEED_ADMIN
        xSemaphoreTake(mtx, portMAX_DELAY);
        int cleared = 0, room_cleared = 0;
        for (int i = 0; i < nusers; i++) {
            if (users[i].inside) { users[i].inside = false; cleared++; }
            if (users[i].current_room != 0) { users[i].current_room = 0; users[i].entered_at = 0; room_cleared++; }
        }
        xSemaphoreGive(mtx);
        bool saved = save_users();
        char detail[32]; snprintf(detail, sizeof detail, "cleared=%d rooms=%d", cleared, room_cleared);
        logev("INSIDE_RESET", "-", "admin", detail);
        if (saved) printf("Cleared %d badged occupant(s), %d room presence(s); state saved\n", cleared, room_cleared);
        else if (!sd_ok) printf("Cleared %d badged occupant(s), %d room presence(s) in memory; SD unavailable\n", cleared, room_cleared);
        else printf("Cleared %d badged occupant(s), %d room presence(s) in memory; ERROR: could not save state\n", cleared, room_cleared);
    }
    else if (!strcmp(c, "log")) {
        if (!sd_ok) { puts("SD card not available"); return; }
        int n = ac > 1 ? atoi(av[1]) : 8; if (n < 1) n = 1; if (n > 10) n = 10;
        static char ring[10][96]; int cnt = 0; char l[96];
        xSemaphoreTake(sd_mtx, portMAX_DELAY);
        FILE *f = fopen("/sdcard/access.csv", "r");
        if (f) { while (fgets(l, sizeof l, f)) strcpy(ring[cnt++ % 10], l); fclose(f); }
        xSemaphoreGive(sd_mtx);
        for (int i = cnt > n ? cnt - n : 0; i < cnt; i++) fputs(ring[i % 10], stdout);
    }
    else if (!strcmp(c, "login") && ac == 2) {
        int64_t t = esp_timer_get_time();
        if (t < login_lock) { puts("Login locked for 30 s (too many failures)"); return; }
        char h[65]; hash_pin(av[1], h);
        if (!strcmp(h, pin_hash)) { admin = true; admin_until = t + 60000000LL; login_fails = 0; logev("LOGIN_OK", "-", "admin", "session 60s"); puts("Admin logged in"); }
        else {
            logev("LOGIN_FAIL", "-", "admin", "wrong pin"); puts("Wrong PIN");
            if (++login_fails >= 3) { login_lock = t + 30000000LL; login_fails = 0; logev("ALERT", "-", "admin", "3 bad logins: locked 30s"); alarm_req(1500); }
        }
    }
    else if (!strcmp(c, "logout")) { admin = false; puts("Logged out"); }
    else if (!strcmp(c, "schedule") && ac == 3) {
        NEED_ADMIN
        int o = parse_hm(av[1]), cl = parse_hm(av[2]);
        if (o < 0 || cl < 0) { puts("Use HH:MM HH:MM"); return; }
        xSemaphoreTake(mtx, portMAX_DELAY); open_min = o; close_min = cl; xSemaphoreGive(mtx);
        save_cfg(); char d[24]; snprintf(d, sizeof d, "%s-%s", av[1], av[2]); logev("CONFIG", "-", "schedule", d); puts("Schedule saved");
    }
    else if (!strcmp(c, "adduser") && ac == 3) {
        NEED_ADMIN
        up(av[1]); xSemaphoreTake(mtx, portMAX_DELAY);
        if (find(av[1])) puts("Tag already exists"); else if (nusers >= MAX_USERS) puts("User table full");
        else { add_user(av[1], av[2], false); puts("User added"); }
        xSemaphoreGive(mtx); save_users(); logev("CONFIG", av[1], av[2], "user added");
    }
    else if (!strcmp(c, "deluser") && ac == 2) {
        NEED_ADMIN
        up(av[1]); xSemaphoreTake(mtx, portMAX_DELAY); user_t *u = find(av[1]);
        if (u) {
            char rmuid[12], rmname[12]; int in_room = u->current_room;
            snprintf(rmuid, sizeof rmuid, "%s", u->uid); snprintf(rmname, sizeof rmname, "%s", u->name);
            if (in_room != 0) {
                *u = users[--nusers];
                int after = count_in_room(in_room);
                char detail[128]; snprintf(detail, sizeof detail, "Room%d count=%d (removed)", in_room, after);
                logev("ROOM_OUT", rmuid, rmname, detail);
            } else {
                *u = users[--nusers];
            }
            puts("User removed");
        } else puts("No such tag");
        xSemaphoreGive(mtx); save_users(); logev("CONFIG", av[1], "-", "user removed");
    }
    else if (!strcmp(c, "evac") && ac == 2) {
        NEED_ADMIN
        evac = !strcmp(av[1], "on"); logev("EVAC", "-", "admin", evac ? "ON: exits open" : "OFF"); if (evac) alarm_req(2000);
    }
    else if (!strcmp(c, "passwd") && ac == 2 && strlen(av[1]) >= 4) { NEED_ADMIN hash_pin(av[1], pin_hash); save_cfg(); logev("CONFIG", "-", "admin", "pin changed"); puts("PIN changed"); }
    else if (!strcmp(c, "settime") && ac == 2) {
        NEED_ADMIN
        int m = parse_hm(av[1]); if (m < 0) { puts("Use HH:MM"); return; }
        set_clock(m * 60, speed);
        /* clock jump: room presence entries are no longer time-valid */
        xSemaphoreTake(mtx, portMAX_DELAY);
        for (int i = 0; i < nusers; i++) {
            if (users[i].current_room) { users[i].current_room = 0; users[i].entered_at = 0; }
        }
        xSemaphoreGive(mtx);
        logev("CONFIG", "-", "system", "room presence cleared on clock jump");
        puts("Time set");
    }
    else if (!strcmp(c, "speed") && ac == 2) {
        NEED_ADMIN
        int v = atoi(av[1]); if (v < 1 || v > 600) { puts("1-600"); return; }
        set_clock(now_s(), v); printf("Clock speed x%d\n", v);
    }
    else if (!strcmp(c, "maxocc") && ac == 3) {
        NEED_ADMIN
        int room = atoi(av[1]); int n = atoi(av[2]);
        if (room < 1 || room > 2 || n < 1 || n > MAX_USERS) { puts("Use: maxocc 1|2 <1-500>"); return; }
        xSemaphoreTake(mtx, portMAX_DELAY); maxocc[room - 1] = n; xSemaphoreGive(mtx);
        save_cfg(); char d[24]; snprintf(d, sizeof d, "room%d=%d", room, n); logev("CONFIG", "-", "maxocc", d);
        printf("Room%d capacity set to %d\n", room, n);
    }
    else if (!strcmp(c, "resetpeak") && ac == 1) {
        NEED_ADMIN
        xSemaphoreTake(mtx, portMAX_DELAY);
        peak_count[0] = peak_count[1] = 0; peak_at_s[0] = peak_at_s[1] = 0;
        xSemaphoreGive(mtx);
        logev("CONFIG", "-", "admin", "peaks reset"); puts("Peaks reset");
    }
    else puts("Unknown command. Type: help");
}
static void cli_task(void *a) {
    char line[64]; int n = 0; uint8_t ch;
    printf("\nCoCIS Smart System ready. Type 'help'.\n> "); fflush(stdout);
    for (;;) {
        if (uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY) <= 0) continue;
        if (ch == '\r' || ch == '\n') {
            if (n) { line[n] = 0; printf("\n"); exec(line); n = 0; printf("> "); fflush(stdout); }
        } else if ((ch == 8 || ch == 127) && n) { n--; printf("\b \b"); fflush(stdout); }
        else if (ch >= 32 && ch < 127 && n < 63) { line[n++] = ch; putchar(ch); fflush(stdout); }
    }
}

/* ---------- boot ---------- */
void app_main(void) {
    if (psa_crypto_init() != PSA_SUCCESS) {
        printf("ERROR: cryptography initialization failed\n");
        return;
    }
    mtx = xSemaphoreCreateMutex(); sd_mtx = xSemaphoreCreateMutex();
    log_q = xQueueCreate(16, 96); alarm_q = xQueueCreate(4, sizeof(int));
    gate_q[0] = xQueueCreate(4, 12); gate_q[1] = xQueueCreate(4, 12);
    room_report_q = xQueueCreate(8, sizeof(room_report_t));
    if (!mtx || !sd_mtx || !log_q || !alarm_q || !gate_q[0] || !gate_q[1] || !room_report_q) {
        printf("ERROR: unable to allocate system queues/mutexes\n");
        return;
    }
    for (int r = 0; r < 2; r++) room[r].last_motion_us = -1000000000000LL;

    const bool entrance = APP_ROLE == ROLE_ENTRANCE;
    const bool demo = APP_ROLE == ROLE_DEMO;
    const bool room_node = APP_ROLE == ROLE_ROOM1 || APP_ROLE == ROLE_ROOM2;
    const int local_room = APP_ROLE == ROLE_ROOM2 ? 1 : 0;
    uint64_t output_mask = 0, input_mask = 0;
    if (entrance || demo) output_mask |= (1ULL << PIN_DOOR_IN) | (1ULL << PIN_DOOR_OUT) | (1ULL << PIN_RED);
    if (room_node) {
        output_mask |= 1ULL << LIGHT[local_room];
        input_mask |= 1ULL << PIR[local_room];
    } else if (demo) {
        output_mask |= (1ULL << LIGHT[0]) | (1ULL << LIGHT[1]);
        input_mask |= (1ULL << PIR[0]) | (1ULL << PIR[1]);
    }
    if (output_mask) {
        gpio_config_t o = { .pin_bit_mask = output_mask, .mode = GPIO_MODE_OUTPUT };
        ESP_ERROR_CHECK(gpio_config(&o));
        for (int pin = 0; pin <= 39; pin++)
            if (output_mask & (1ULL << pin)) gpio_set_level(pin, 0);
    }
    if (input_mask) {
        gpio_config_t i = { .pin_bit_mask = input_mask, .mode = GPIO_MODE_INPUT };
        ESP_ERROR_CHECK(gpio_config(&i));
    }

    if (entrance || demo) {
        ledc_timer_config_t lt = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_8_BIT,
                                   .timer_num = LEDC_TIMER_0, .freq_hz = 2000, .clk_cfg = LEDC_AUTO_CLK };
        ESP_ERROR_CHECK(ledc_timer_config(&lt));
        ledc_channel_config_t lc = { .gpio_num = PIN_BUZZ, .speed_mode = LEDC_LOW_SPEED_MODE,
                                     .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0,
                                     .duty = 0, .hpoint = 0 };
        ESP_ERROR_CHECK(ledc_channel_config(&lc));
    }

    set_clock(8 * 3600, 1);
    sd_ok = (entrance || demo) && sd_init();
    if (entrance || demo) {
        printf("SD card: %s\n", sd_ok ? "mounted" : "NOT FOUND (running without storage)");
        load_all();
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    }

    if (!demo) {
        bool radio_ok = radio_init(entrance);
        printf("ESP-NOW link: %s (channel 1, broadcast heartbeat)\n", radio_ok ? "ready" : "unavailable");
        if (entrance && radio_ok) xTaskCreate(radio_report_task, "room_reports", 3072, NULL, 4, NULL);
    }

    if (entrance || demo) {
        xTaskCreate(logger_task, "logger", 4096, NULL, 4, NULL);
        xTaskCreate(alarm_task, "alarm", 2048, NULL, 4, NULL);
        xTaskCreate(gate_task, "gate_in", 4096, (void *)0, 5, NULL);
        xTaskCreate(gate_task, "gate_out", 4096, (void *)1, 5, NULL);
        xTaskCreate(sched_task, "sched", 3072, NULL, 3, NULL);
        xTaskCreate(cli_task, "cli", 6144, NULL, 5, NULL);
    }
    if (room_node) {
        printf("CoCIS %s terminal: PIR GPIO%d, light GPIO%d\n",
               local_room == 0 ? "Room 1" : "Room 2", PIR[local_room], LIGHT[local_room]);
        xTaskCreate(room_task, "room_monitor", 3072, (void *)(intptr_t)local_room, 5, NULL);
    } else if (demo) {
        xTaskCreate(room_task, "room1", 3072, (void *)0, 5, NULL);
        xTaskCreate(room_task, "room2", 3072, (void *)1, 5, NULL);
    }
}
