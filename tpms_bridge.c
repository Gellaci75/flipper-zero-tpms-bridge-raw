#include "tpms_bridge.h"
#include "tpms_decoder.h"
#include "tpms_lf.h"
#include "tpms_view.h"

#include <furi.h>
#include <furi_hal.h>
#include <flipper_format/flipper_format_i.h>
#include <gui/gui.h>
#include <gui/modules/text_input.h>
#include <gui/modules/validators.h>
#include <gui/view_port.h>
#include <gui/view_holder.h>
#include <input/input.h>
#include <applications/drivers/subghz/cc1101_ext/cc1101_ext_interconnect.h>
#include <lib/subghz/devices/cc1101_int/cc1101_int_interconnect.h>
#include <lib/subghz/devices/devices.h>
#include <lib/subghz/blocks/generic.h>
#include <lib/subghz/environment.h>
#include <lib/subghz/protocols/protocol_items.h>
#include <lib/subghz/protocols/raw.h>
#include <lib/subghz/subghz_protocol_registry.h>
#include <lib/subghz/transmitter.h>
#include <notification/notification_messages.h>
#include <power/power_service/power.h>
#include <storage/storage.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define TAG "TpmsBridge"

#define TPMS_INPUT_QUEUE_SIZE 8

/** Pause before retrying to claim the radio, ms. */
#define TPMS_RADIO_RETRY_MS 1000
#define TPMS_RADIO_SWITCH_DELAY_MS 1000
#define TPMS_TX_COOLDOWN_MS 1000
#define TPMS_TX_DISPLAY_HOLD_MS 1200

/* Bounded pre-trigger pulse history. A decoded TPMS frame triggers a save.
 * This is demodulated level/duration data, NOT RF IQ samples. */
#define TPMS_RAW_CAPACITY 2048
#define TPMS_RAW_MAX_FILES 100
#define TPMS_RAW_DIR "/ext/subghz/TPMS_Bridge_RAW"
#define TPMS_RAW_HELP_PAGES 5
#define TPMS_RAW_POST_CAPTURE_MS 250
#define TPMS_RAW_DUPLICATE_HOLD_MS 30000

typedef enum {
    TpmsRawActionOpen,
    TpmsRawActionPlay,
    TpmsRawActionRawInfo,
    TpmsRawActionRename,
    TpmsRawActionDelete,
    TpmsRawActionDeleteAll,
    TpmsRawActionCount,
} TpmsRawAction;

typedef struct {
    int32_t pulses[TPMS_RAW_CAPACITY];
    int32_t snapshot[TPMS_RAW_CAPACITY];
    size_t head;
    size_t count;
    size_t snapshot_count;
    uint32_t frequency;
    TpmsModulation modulation;
    TpmsFrame frame;
    int16_t rssi_x10;
    bool has_frame;
    bool post_capture;
    uint32_t post_until_tick;
    uint8_t last_saved_protocol;
    uint32_t last_saved_id;
    uint32_t last_saved_tick;
    bool last_saved_valid;
    uint16_t next_file;
    bool pending;
} TpmsRawCapture;

static void tpms_raw_interval(bool level, uint32_t duration, void* context) {
    TpmsRawCapture* capture = context;
    if(duration == 0 || duration > 1000000) return;
    capture->pulses[capture->head] = level ? (int32_t)duration : -(int32_t)duration;
    capture->head = (capture->head + 1) % TPMS_RAW_CAPACITY;
    if(capture->count < TPMS_RAW_CAPACITY) capture->count++;
}

static void tpms_raw_frame(const TpmsFrame* frame, float rssi, void* context) {
    TpmsRawCapture* capture = context;
    if(capture->pending || capture->post_capture || !capture->count) return;
    const uint32_t now = furi_get_tick();
    if(capture->last_saved_valid && capture->last_saved_protocol == frame->protocol &&
       capture->last_saved_id == frame->id &&
       now - capture->last_saved_tick < furi_ms_to_ticks(TPMS_RAW_DUPLICATE_HOLD_MS)) {
        return;
    }
    capture->frame = *frame;
    capture->rssi_x10 = (int16_t)(rssi * 10.0f);
    capture->has_frame = true;
    capture->post_capture = true;
    capture->post_until_tick = now + furi_ms_to_ticks(TPMS_RAW_POST_CAPTURE_MS);
}

static void tpms_raw_finish_post_capture(TpmsRawCapture* capture) {
    if(!capture->post_capture || capture->pending || !capture->count) return;
    if((int32_t)(furi_get_tick() - capture->post_until_tick) < 0) return;

    capture->snapshot_count = capture->count;
    const size_t start = (capture->head + TPMS_RAW_CAPACITY - capture->count) % TPMS_RAW_CAPACITY;
    for(size_t i = 0; i < capture->count; i++) {
        capture->snapshot[i] = capture->pulses[(start + i) % TPMS_RAW_CAPACITY];
    }
    capture->post_capture = false;
    capture->pending = true;
}

static void tpms_raw_clear(TpmsRawCapture* capture) {
    capture->head = 0;
    capture->count = 0;
    capture->pending = false;
    capture->has_frame = false;
    capture->post_capture = false;
    capture->post_until_tick = 0;
}

static void tpms_raw_make_timestamp_name(
    char* name,
    size_t name_size,
    const char* prefix,
    uint8_t suffix) {
    DateTime datetime;
    furi_hal_rtc_get_datetime(&datetime);
    if(suffix == 0) {
        snprintf(
            name,
            name_size,
            "%s_%02u.%02u.%02u_%02u%02u.sub",
            prefix,
            (uint8_t)(datetime.year % 100),
            datetime.month,
            datetime.day,
            datetime.hour,
            datetime.minute);
    } else {
        snprintf(
            name,
            name_size,
            "%s_%02u.%02u.%02u_%02u%02u_%02u.sub",
            prefix,
            (uint8_t)(datetime.year % 100),
            datetime.month,
            datetime.day,
            datetime.hour,
            datetime.minute,
            suffix);
    }
}

static void tpms_raw_save(TpmsRawCapture* capture) {
    if(!capture->pending || capture->next_file >= TPMS_RAW_MAX_FILES) return;
    capture->pending = false;
    Storage* storage = furi_record_open(RECORD_STORAGE);
    storage_common_mkdir(storage, TPMS_RAW_DIR);
    File* file = storage_file_alloc(storage);
    char name[TPMS_RAW_NAME_MAX];
    char path[96];
    bool opened = false;
    for(uint8_t suffix = 0; suffix < 100; suffix++) {
        tpms_raw_make_timestamp_name(name, sizeof(name), "RAW", suffix);
        snprintf(path, sizeof(path), TPMS_RAW_DIR "/%s", name);
        if(storage_file_open(file, path, FSAM_WRITE, FSOM_CREATE_NEW)) {
            opened = true;
            break;
        }
    }
    if(opened) {
        char line[96];
#define RAW_WRITE(str) storage_file_write(file, str, strlen(str))
        RAW_WRITE("Filetype: Flipper SubGhz RAW File\nVersion: 1\n");
        snprintf(line, sizeof(line), "Frequency: %lu\n", (unsigned long)capture->frequency);
        RAW_WRITE(line);
        if(capture->modulation == TpmsModulationOok) {
            RAW_WRITE("Preset: FuriHalSubGhzPresetOok270Async\n");
        } else {
            /* FSK custom preset bytes are firmware-specific; this file is
             * a diagnostic capture and may not be directly replayable. */
            RAW_WRITE("Preset: FuriHalSubGhzPreset2FSKDev476Async\n");
        }
        RAW_WRITE("Protocol: RAW\n");
        if(capture->has_frame) {
            const TpmsFrame* frame = &capture->frame;
            snprintf(line, sizeof(line), "TPMSProtocol: %u\n", (unsigned)frame->protocol);
            RAW_WRITE(line);
            snprintf(line, sizeof(line), "TPMSID: %lu\n", (unsigned long)frame->id);
            RAW_WRITE(line);
            snprintf(line, sizeof(line), "TPMSHave: %u\n", (unsigned)frame->have);
            RAW_WRITE(line);
            snprintf(line, sizeof(line), "TPMSFlags: %u\n", (unsigned)frame->flags);
            RAW_WRITE(line);
            snprintf(
                line,
                sizeof(line),
                "TPMSPressureKpaX100: %ld\n",
                (long)frame->pressure_kpa_x100);
            RAW_WRITE(line);
            snprintf(
                line,
                sizeof(line),
                "TPMSTemperatureC: %d\n",
                (int)frame->temperature_c);
            RAW_WRITE(line);
            snprintf(line, sizeof(line), "TPMSRssiX10: %d\n", (int)capture->rssi_x10);
            RAW_WRITE(line);
        }
        size_t i = 0;
        while(i < capture->snapshot_count) {
            RAW_WRITE("RAW_Data:");
            for(size_t j = 0; j < 64 && i < capture->snapshot_count; j++, i++) {
                snprintf(line, sizeof(line), " %ld", (long)capture->snapshot[i]);
                RAW_WRITE(line);
            }
            RAW_WRITE("\n");
        }
        storage_file_close(file);
        if(capture->has_frame) {
            capture->last_saved_protocol = capture->frame.protocol;
            capture->last_saved_id = capture->frame.id;
            capture->last_saved_tick = furi_get_tick();
            capture->last_saved_valid = true;
        }
    }
#undef RAW_WRITE
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
}

static bool tpms_raw_name_is_sub(const char* name) {
    const char* dot = strrchr(name, '.');
    return dot && strcmp(dot, ".sub") == 0;
}

static void tpms_raw_build_path(char* path, size_t path_size, const char* name) {
    snprintf(path, path_size, TPMS_RAW_DIR "/%s", name);
}

static void tpms_raw_clear_file_tpms(TpmsBridgeApp* app) {
    app->raw_file_tpms_valid = false;
    app->raw_file_tpms_protocol = 0;
    app->raw_file_tpms_id = 0;
    app->raw_file_tpms_pressure_kpa_x100 = 0;
    app->raw_file_tpms_temperature_c = 0;
    app->raw_file_tpms_rssi_x10 = 0;
    app->raw_file_tpms_have = 0;
    app->raw_file_tpms_flags = 0;
}

static void tpms_raw_store_file_frame(TpmsBridgeApp* app, const TpmsFrame* frame) {
    app->raw_file_tpms_valid = true;
    app->raw_file_tpms_protocol = frame->protocol;
    app->raw_file_tpms_id = frame->id;
    app->raw_file_tpms_pressure_kpa_x100 = frame->pressure_kpa_x100;
    app->raw_file_tpms_temperature_c = frame->temperature_c;
    app->raw_file_tpms_have = frame->have;
    app->raw_file_tpms_flags = frame->flags;
}

static FuriHalSubGhzPreset tpms_raw_preset_from_name(const char* preset_name) {
    if(!strcmp(preset_name, "FuriHalSubGhzPresetOok270Async")) {
        return FuriHalSubGhzPresetOok270Async;
    } else if(!strcmp(preset_name, "FuriHalSubGhzPresetOok650Async")) {
        return FuriHalSubGhzPresetOok650Async;
    } else if(!strcmp(preset_name, "FuriHalSubGhzPreset2FSKDev238Async")) {
        return FuriHalSubGhzPreset2FSKDev238Async;
    } else if(!strcmp(preset_name, "FuriHalSubGhzPreset2FSKDev12KAsync")) {
        return FuriHalSubGhzPreset2FSKDev12KAsync;
    } else if(!strcmp(preset_name, "FuriHalSubGhzPreset2FSKDev476Async")) {
        return FuriHalSubGhzPreset2FSKDev476Async;
    } else if(!strcmp(preset_name, "FuriHalSubGhzPresetCustom")) {
        return FuriHalSubGhzPresetCustom;
    }
    return FuriHalSubGhzPresetIDLE;
}

static SubGhzEnvironment* tpms_raw_environment_alloc(void) {
    SubGhzEnvironment* environment = subghz_environment_alloc();
    subghz_environment_set_protocol_registry(environment, (void*)&subghz_protocol_registry);
    return environment;
}

static void tpms_raw_follow_file_selection(TpmsBridgeApp* app) {
    if(app->raw_file_count == 0) {
        app->raw_file_selected = 0;
        app->raw_file_scroll = 0;
        return;
    }

    if(app->raw_file_selected >= app->raw_file_count) {
        app->raw_file_selected = app->raw_file_count - 1;
    }

    if(app->raw_file_selected < app->raw_file_scroll) {
        app->raw_file_scroll = app->raw_file_selected;
    } else if(app->raw_file_selected >= app->raw_file_scroll + TPMS_RAW_FILE_ROWS) {
        app->raw_file_scroll = app->raw_file_selected - TPMS_RAW_FILE_ROWS + 1;
    }

    if(app->raw_file_count <= TPMS_RAW_FILE_ROWS) app->raw_file_scroll = 0;
}

static void tpms_raw_refresh_files(TpmsBridgeApp* app) {
    app->raw_file_count = 0;
    app->raw_file_size = 0;

    Storage* storage = furi_record_open(RECORD_STORAGE);
    File* dir = storage_file_alloc(storage);
    if(storage_dir_open(dir, TPMS_RAW_DIR)) {
        FileInfo info;
        char name[TPMS_RAW_NAME_MAX];
        while(app->raw_file_count < TPMS_RAW_FILE_MAX &&
              storage_dir_read(dir, &info, name, sizeof(name))) {
            if(file_info_is_dir(&info) || !tpms_raw_name_is_sub(name)) continue;

            strlcpy(
                app->raw_file_names[app->raw_file_count],
                name,
                sizeof(app->raw_file_names[app->raw_file_count]));
            app->raw_file_count++;
        }
    }
    storage_dir_close(dir);
    storage_file_free(dir);
    furi_record_close(RECORD_STORAGE);

    tpms_raw_follow_file_selection(app);
}

static void tpms_raw_update_selected_info(TpmsBridgeApp* app) {
    app->raw_file_size = 0;
    app->raw_file_frequency = 0;
    app->raw_file_protocol[0] = '\0';
    tpms_raw_clear_file_tpms(app);
    strlcpy(app->raw_message, "No TPMS meta", sizeof(app->raw_message));
    if(app->raw_file_count == 0) return;

    char path[96];
    tpms_raw_build_path(path, sizeof(path), app->raw_file_names[app->raw_file_selected]);

    Storage* storage = furi_record_open(RECORD_STORAGE);
    FileInfo info;
    if(storage_common_stat(storage, path, &info) == FSE_OK && !file_info_is_dir(&info)) {
        app->raw_file_size = info.size;
    }

    FlipperFormat* file = flipper_format_file_alloc(storage);
    FuriString* text = furi_string_alloc();
    uint32_t version = 0;
    uint32_t value = 0;
    int32_t signed_value = 0;
    do {
        if(!flipper_format_file_open_existing(file, path)) break;
        if(!flipper_format_read_header(file, text, &version)) break;
        flipper_format_read_uint32(file, "Frequency", &app->raw_file_frequency, 1);
        if(flipper_format_read_string(file, "Protocol", text)) {
            strlcpy(app->raw_file_protocol, furi_string_get_cstr(text), sizeof(app->raw_file_protocol));
        }
        TpmsFrame frame;
        memset(&frame, 0, sizeof(frame));
        if(flipper_format_read_uint32(file, "TPMSProtocol", &value, 1)) {
            frame.protocol = (uint8_t)value;
            if(flipper_format_read_uint32(file, "TPMSID", &value, 1)) frame.id = value;
            if(flipper_format_read_uint32(file, "TPMSHave", &value, 1)) frame.have = (uint8_t)value;
            if(flipper_format_read_uint32(file, "TPMSFlags", &value, 1)) frame.flags = (uint8_t)value;
            if(flipper_format_read_int32(file, "TPMSPressureKpaX100", &signed_value, 1)) {
                frame.pressure_kpa_x100 = signed_value;
            }
            if(flipper_format_read_int32(file, "TPMSTemperatureC", &signed_value, 1)) {
                frame.temperature_c = (int16_t)signed_value;
            }
            tpms_raw_store_file_frame(app, &frame);
            if(flipper_format_read_int32(file, "TPMSRssiX10", &signed_value, 1)) {
                app->raw_file_tpms_rssi_x10 = (int16_t)signed_value;
            }
            strlcpy(app->raw_message, "Meta OK", sizeof(app->raw_message));
        }
    } while(false);
    furi_string_free(text);
    flipper_format_free(file);
    furi_record_close(RECORD_STORAGE);
}

typedef struct {
    bool found;
    TpmsFrame frame;
} TpmsRawDecodeContext;

static void tpms_raw_decode_callback(const TpmsFrame* frame, void* context) {
    TpmsRawDecodeContext* decode = context;
    if(decode->found) return;
    decode->frame = *frame;
    decode->found = true;
}

static void tpms_raw_decode_line(TpmsDecoder* decoder, const char* line) {
    if(strncmp(line, "RAW_Data:", 9) != 0) return;

    const char* cursor = line + 9;
    while(*cursor) {
        while(*cursor == ' ' || *cursor == '\t') cursor++;
        if(!*cursor) break;

        char* end = NULL;
        const long value = strtol(cursor, &end, 10);
        if(end == cursor) break;

        const bool level = value > 0;
        const uint32_t duration = (uint32_t)(value < 0 ? -value : value);
        if(duration > 0) tpms_decoder_feed(decoder, level, duration);
        cursor = end;
    }
}

static bool tpms_raw_decode_path(const char* path, TpmsModulation modulation, TpmsFrame* out_frame) {
    TpmsRawDecodeContext decode = {0};
    TpmsDecoder* decoder = tpms_decoder_alloc(tpms_raw_decode_callback, &decode);
    tpms_decoder_set_modulation(decoder, modulation);

    Storage* storage = furi_record_open(RECORD_STORAGE);
    File* file = storage_file_alloc(storage);
    bool opened = storage_file_open(file, path, FSAM_READ, FSOM_OPEN_EXISTING);

    char buffer[128];
    char line[512];
    size_t line_len = 0;
    while(opened && !decode.found) {
        const size_t read = storage_file_read(file, buffer, sizeof(buffer));
        if(read == 0) break;

        for(size_t i = 0; i < read && !decode.found; i++) {
            const char c = buffer[i];
            if(c == '\r') continue;
            if(c == '\n') {
                line[line_len] = '\0';
                tpms_raw_decode_line(decoder, line);
                line_len = 0;
            } else if(line_len + 1 < sizeof(line)) {
                line[line_len++] = c;
            }
        }
    }
    if(opened && !decode.found && line_len > 0) {
        line[line_len] = '\0';
        tpms_raw_decode_line(decoder, line);
    }

    if(opened) storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);

    if(decode.found && out_frame) *out_frame = decode.frame;
    tpms_decoder_free(decoder);
    return decode.found;
}

static bool tpms_raw_decode_selected(TpmsBridgeApp* app) {
    if(app->raw_file_count == 0) return false;

    tpms_raw_update_selected_info(app);
    if(app->raw_file_tpms_valid) {
        strlcpy(app->raw_message, "Meta OK", sizeof(app->raw_message));
        return true;
    }

    char path[96];
    tpms_raw_build_path(path, sizeof(path), app->raw_file_names[app->raw_file_selected]);

    Storage* storage = furi_record_open(RECORD_STORAGE);
    FlipperFormat* file = flipper_format_file_alloc(storage);
    FuriString* text = furi_string_alloc();
    TpmsModulation modulation = TpmsModulationFsk;
    do {
        if(!flipper_format_file_open_existing(file, path)) break;
        if(flipper_format_read_string(file, "Preset", text)) {
            if(strstr(furi_string_get_cstr(text), "Ook")) modulation = TpmsModulationOok;
        }
    } while(false);
    furi_string_free(text);
    flipper_format_free(file);
    furi_record_close(RECORD_STORAGE);

    TpmsFrame frame;
    memset(&frame, 0, sizeof(frame));
    if(tpms_raw_decode_path(path, modulation, &frame)) {
        tpms_raw_store_file_frame(app, &frame);
        app->raw_file_tpms_rssi_x10 = 0;
        strlcpy(app->raw_message, "Decoded", sizeof(app->raw_message));
        return true;
    }

    strlcpy(app->raw_message, "No TPMS data", sizeof(app->raw_message));
    return false;
}

static bool tpms_raw_delete_selected(TpmsBridgeApp* app) {
    if(app->raw_file_count == 0) return false;

    char path[96];
    tpms_raw_build_path(path, sizeof(path), app->raw_file_names[app->raw_file_selected]);

    Storage* storage = furi_record_open(RECORD_STORAGE);
    const bool ok = storage_common_remove(storage, path) == FSE_OK;
    furi_record_close(RECORD_STORAGE);

    if(ok) {
        strlcpy(app->raw_message, "Deleted", sizeof(app->raw_message));
        tpms_raw_refresh_files(app);
    } else {
        strlcpy(app->raw_message, "Delete failed", sizeof(app->raw_message));
    }
    return ok;
}
static bool tpms_raw_find_first_sub(Storage* storage, char* name, size_t name_size) {
    File* dir = storage_file_alloc(storage);
    FileInfo info;
    bool found = false;

    if(storage_dir_open(dir, TPMS_RAW_DIR)) {
        while(storage_dir_read(dir, &info, name, name_size)) {
            if(!file_info_is_dir(&info) && tpms_raw_name_is_sub(name)) {
                found = true;
                break;
            }
        }
        storage_dir_close(dir);
    }

    storage_file_free(dir);
    return found;
}

static bool tpms_raw_delete_all(TpmsBridgeApp* app) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    char name[TPMS_RAW_NAME_MAX];
    char path[96];
    uint32_t deleted = 0;
    bool failed = false;

    while(tpms_raw_find_first_sub(storage, name, sizeof(name))) {
        tpms_raw_build_path(path, sizeof(path), name);
        if(storage_common_remove(storage, path) != FSE_OK) {
            failed = true;
            break;
        }
        deleted++;
    }

    furi_record_close(RECORD_STORAGE);
    tpms_raw_refresh_files(app);

    if(failed) {
        strlcpy(app->raw_message, "Delete failed", sizeof(app->raw_message));
        return false;
    }
    if(deleted == 0) {
        strlcpy(app->raw_message, "No files", sizeof(app->raw_message));
        return false;
    }

    snprintf(app->raw_message, sizeof(app->raw_message), "Deleted %lu", (unsigned long)deleted);
    return true;
}

typedef struct {
    FuriSemaphore* done;
    bool accepted;
} TpmsRenameCtx;

static void tpms_raw_rename_done(void* context) {
    TpmsRenameCtx* ctx = context;
    ctx->accepted = true;
    furi_semaphore_release(ctx->done);
}

static void tpms_raw_rename_back(void* context) {
    TpmsRenameCtx* ctx = context;
    ctx->accepted = false;
    furi_semaphore_release(ctx->done);
}

static void tpms_raw_name_without_ext(char* out, size_t out_size, const char* name) {
    strlcpy(out, name, out_size);
    char* dot = strrchr(out, '.');
    if(dot && !strcmp(dot, ".sub")) *dot = '\0';
}

static bool tpms_raw_ask_rename(TpmsBridgeApp* app, const char* current_name) {
    char current_base[TPMS_RAW_RENAME_MAX];
    tpms_raw_name_without_ext(current_base, sizeof(current_base), current_name);
    strlcpy(app->rename_buffer, current_base, sizeof(app->rename_buffer));

    gui_remove_view_port(app->gui, app->view_port);

    TpmsRenameCtx ctx = {.done = furi_semaphore_alloc(1, 0), .accepted = false};
    TextInput* text_input = text_input_alloc();
    text_input_set_header_text(text_input, "Rename .sub file");
    text_input_set_result_callback(
        text_input, tpms_raw_rename_done, &ctx, app->rename_buffer, sizeof(app->rename_buffer), false);

    ValidatorIsFile* validator = validator_is_file_alloc_init(TPMS_RAW_DIR, ".sub", current_base);
    text_input_set_validator(text_input, validator_is_file_callback, validator);

    ViewHolder* view_holder = view_holder_alloc();
    view_holder_attach_to_gui(view_holder, app->gui);
    view_holder_set_back_callback(view_holder, tpms_raw_rename_back, &ctx);
    view_holder_set_view(view_holder, text_input_get_view(text_input));

    furi_semaphore_acquire(ctx.done, FuriWaitForever);

    view_holder_set_view(view_holder, NULL);
    view_holder_free(view_holder);
    text_input_free(text_input);
    validator_is_file_free(validator);
    furi_semaphore_free(ctx.done);

    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);
    return ctx.accepted;
}

static bool tpms_raw_rename_selected(TpmsBridgeApp* app) {
    if(app->raw_file_count == 0) return false;

    char old_name[TPMS_RAW_NAME_MAX];
    strlcpy(old_name, app->raw_file_names[app->raw_file_selected], sizeof(old_name));
    if(!tpms_raw_ask_rename(app, old_name)) {
        strlcpy(app->raw_message, "Rename cancelled", sizeof(app->raw_message));
        return false;
    }

    char old_path[96];
    char new_name[TPMS_RAW_RENAME_MAX + 5];
    char new_path[128];
    tpms_raw_build_path(old_path, sizeof(old_path), old_name);
    snprintf(new_name, sizeof(new_name), "%s.sub", app->rename_buffer);
    snprintf(new_path, sizeof(new_path), TPMS_RAW_DIR "/%s", new_name);

    Storage* storage = furi_record_open(RECORD_STORAGE);
    const bool ok = storage_common_rename(storage, old_path, new_path) == FSE_OK;
    furi_record_close(RECORD_STORAGE);

    if(ok) {
        tpms_raw_refresh_files(app);
        for(uint8_t i = 0; i < app->raw_file_count; i++) {
            if(strcmp(app->raw_file_names[i], new_name) == 0) {
                app->raw_file_selected = i;
                tpms_raw_follow_file_selection(app);
                break;
            }
        }
    } else {
        strlcpy(app->raw_message, "Rename failed", sizeof(app->raw_message));
    }
    return ok;
}

typedef struct {
    const SubGhzDevice* device;
    bool device_begun;
    bool device_external;
    bool external_power_owned;
    bool power_suppressed;
    bool tx_started;
    uint32_t frequency;
    uint8_t* custom_preset_data;
    SubGhzTransmitter* transmitter;
    SubGhzEnvironment* environment;
} TpmsRawTx;

static void tpms_raw_tx_free(TpmsRawTx* tx) {
    if(!tx) return;
    subghz_block_generic_global.endless_tx = false;
    if(tx->tx_started && tx->device) {
        subghz_devices_stop_async_tx(tx->device);
        tx->tx_started = false;
    }
    if(tx->transmitter) {
        subghz_transmitter_stop(tx->transmitter);
        subghz_transmitter_free(tx->transmitter);
        tx->transmitter = NULL;
    }
    if(tx->custom_preset_data) free(tx->custom_preset_data);
    if(tx->device) {
        subghz_devices_idle(tx->device);
        subghz_devices_sleep(tx->device);
    }
    if(tx->device_external && tx->device_begun) {
        subghz_devices_end(tx->device);
    }
    if(tx->external_power_owned) {
        Power* power = furi_record_open(RECORD_POWER);
        power_enable_otg(power, false);
        furi_record_close(RECORD_POWER);
    }
    if(tx->environment) subghz_environment_free(tx->environment);
    if(tx->power_suppressed) furi_hal_power_suppress_charge_exit();
    free(tx);
}

static void tpms_radio_external_power_on(TpmsRawTx* tx) {
    Power* power = furi_record_open(RECORD_POWER);
    if(!power_is_otg_enabled(power) && !furi_hal_power_is_otg_enabled()) {
        power_enable_otg(power, true);
        tx->external_power_owned = true;
    }
    furi_record_close(RECORD_POWER);
}

static void tpms_radio_external_power_off(TpmsRawTx* tx) {
    if(!tx->external_power_owned) return;
    Power* power = furi_record_open(RECORD_POWER);
    power_enable_otg(power, false);
    furi_record_close(RECORD_POWER);
    tx->external_power_owned = false;
}

static void tpms_raw_choose_device(TpmsRawTx* tx) {
    tx->device = NULL;
    tx->device_external = false;
    tx->device_begun = false;

    tpms_radio_external_power_on(tx);
    const SubGhzDevice* device = subghz_devices_get_by_name(SUBGHZ_DEVICE_CC1101_EXT_NAME);
    if(device == NULL) {
        FURI_LOG_E(TAG, "No %s driver loaded", SUBGHZ_DEVICE_CC1101_EXT_NAME);
    } else {
        const bool external_connected = subghz_devices_is_connect(device);
        if(external_connected && subghz_devices_begin(device)) {
            tx->device = device;
            tx->device_external = true;
            tx->device_begun = true;
            return;
        }
        if(external_connected) subghz_devices_end(device);
        FURI_LOG_W(TAG, "External radio unavailable, using internal");
    }

    tpms_radio_external_power_off(tx);
    tx->device = subghz_devices_get_by_name(SUBGHZ_DEVICE_CC1101_INT_NAME);
}

static bool tpms_raw_start_async_tx(TpmsRawTx* tx) {
    subghz_block_generic_global.endless_tx = true;
    if(subghz_devices_start_async_tx(tx->device, subghz_transmitter_yield, tx->transmitter)) {
        tx->tx_started = true;
        return true;
    }
    subghz_block_generic_global.endless_tx = false;
    return false;
}

static TpmsRawTx* tpms_raw_tx_alloc(const char* path, char* message, size_t message_size) {
    TpmsRawTx* tx = malloc(sizeof(TpmsRawTx));
    memset(tx, 0, sizeof(TpmsRawTx));

    uint32_t temp_data32 = 0;
    uint32_t repeat = 1;
    uint32_t frequency = 0;

    Storage* storage = furi_record_open(RECORD_STORAGE);
    FlipperFormat* file = flipper_format_file_alloc(storage);
    FlipperFormat* raw_data = flipper_format_string_alloc();
    FuriString* text = furi_string_alloc();
    tx->environment = tpms_raw_environment_alloc();

    do {
        tpms_raw_choose_device(tx);
        if(!tx->device) {
            strlcpy(message, "No radio device", message_size);
            break;
        }
        if(!flipper_format_file_open_existing(file, path)) {
            strlcpy(message, "Open failed", message_size);
            break;
        }
        if(!flipper_format_read_header(file, text, &temp_data32)) {
            strlcpy(message, "Bad header", message_size);
            break;
        }
        if((strcmp(furi_string_get_cstr(text), SUBGHZ_KEY_FILE_TYPE) != 0) &&
           (strcmp(furi_string_get_cstr(text), SUBGHZ_RAW_FILE_TYPE) != 0)) {
            strlcpy(message, "Wrong file type", message_size);
            break;
        }
        if(temp_data32 != SUBGHZ_KEY_FILE_VERSION) {
            strlcpy(message, "Bad version", message_size);
            break;
        }
        if(!flipper_format_read_uint32(file, "Frequency", &frequency, 1)) {
            strlcpy(message, "No frequency", message_size);
            break;
        }
        tx->frequency = frequency;
        if(!subghz_devices_is_frequency_valid(tx->device, frequency)) {
            strlcpy(message, "Bad frequency", message_size);
            break;
        }
        if(!flipper_format_read_string(file, "Preset", text)) {
            strlcpy(message, "No preset", message_size);
            break;
        }
        subghz_devices_reset(tx->device);

        if(!strcmp(furi_string_get_cstr(text), "FuriHalSubGhzPresetCustom")) {
            if(!flipper_format_get_value_count(file, "Custom_preset_data", &temp_data32) ||
               !temp_data32 || (temp_data32 % 2)) {
                strlcpy(message, "Bad custom preset", message_size);
                break;
            }
            tx->custom_preset_data = malloc(sizeof(uint8_t) * temp_data32);
            if(!flipper_format_read_hex(file, "Custom_preset_data", tx->custom_preset_data, temp_data32)) {
                strlcpy(message, "Preset read fail", message_size);
                break;
            }
            subghz_devices_load_preset(
                tx->device,
                tpms_raw_preset_from_name(furi_string_get_cstr(text)),
                tx->custom_preset_data);
        } else {
            const FuriHalSubGhzPreset preset = tpms_raw_preset_from_name(furi_string_get_cstr(text));
            if(preset == FuriHalSubGhzPresetIDLE) {
                strlcpy(message, "Unknown preset", message_size);
                break;
            }
            subghz_devices_load_preset(tx->device, preset, NULL);
        }
        subghz_devices_set_frequency(tx->device, frequency);

        if(!flipper_format_read_string(file, "Protocol", text)) {
            strlcpy(message, "No protocol", message_size);
            break;
        }

        if(!strcmp(furi_string_get_cstr(text), "RAW")) {
            subghz_protocol_raw_gen_fff_data(raw_data, path, subghz_devices_get_name(tx->device));
            tx->transmitter = subghz_transmitter_alloc_init(tx->environment, "RAW");
            if(!tx->transmitter ||
               subghz_transmitter_deserialize(tx->transmitter, raw_data) != SubGhzProtocolStatusOk) {
                strlcpy(message, "RAW parse fail", message_size);
                break;
            }
        } else {
            flipper_format_insert_or_update_uint32(file, "Repeat", &repeat, 1);
            tx->transmitter =
                subghz_transmitter_alloc_init(tx->environment, furi_string_get_cstr(text));
            if(!tx->transmitter ||
               subghz_transmitter_deserialize(tx->transmitter, file) != SubGhzProtocolStatusOk) {
                strlcpy(message, "Protocol fail", message_size);
                break;
            }
            flipper_format_delete_key(file, "Repeat");
        }

        furi_hal_power_suppress_charge_enter();
        tx->power_suppressed = true;
        furi_delay_ms(200);
        if(tpms_raw_start_async_tx(tx)) {
            snprintf(message, message_size, "TX %s", tx->device_external ? "EXT" : "INT");
        } else {
            strlcpy(message, "TX restricted", message_size);
            break;
        }
        goto done;
    } while(false);

    tpms_raw_tx_free(tx);
    tx = NULL;

done:
    furi_string_free(text);
    flipper_format_free(raw_data);
    flipper_format_free(file);
    furi_record_close(RECORD_STORAGE);

    return tx;
}

static void tpms_raw_select_file(TpmsBridgeApp* app, int8_t delta) {
    if(delta < 0) {
        if(app->raw_file_selected > 0) app->raw_file_selected--;
    } else if(app->raw_file_count > 0 && app->raw_file_selected + 1 < app->raw_file_count) {
        app->raw_file_selected++;
    }
    tpms_raw_follow_file_selection(app);
}

static void tpms_raw_select_action(TpmsBridgeApp* app, int8_t delta) {
    app->raw_menu_selected =
        (uint8_t)((app->raw_menu_selected + TpmsRawActionCount + delta) % TpmsRawActionCount);
}

void tpms_bridge_report_frame(TpmsBridgeApp* app, const TpmsFrame* frame, float rssi_dbm) {
    furi_check(app);

    furi_mutex_acquire(app->state_mutex, FuriWaitForever);
    tpms_store_update(&app->store, frame, (int16_t)(rssi_dbm * 10.0f), furi_get_tick());
    tpms_view_follow_selection(app);
    furi_mutex_release(app->state_mutex);

    if(app->view_port) view_port_update(app->view_port);
}

void tpms_bridge_tune_radio(TpmsBridgeApp* app, TpmsSession* session) {
    furi_check(app);

    uint8_t slot;
    if(tpms_config_is_auto((TpmsConfig)app->config)) {
        const uint32_t now = furi_get_tick();
        if(app->scan_tick == 0 || now - app->scan_tick > furi_ms_to_ticks(TPMS_SCAN_PERIOD_MS)) {
            app->scan_tick = now;
            app->scan_step =
                (uint8_t)((app->scan_step + 1) % tpms_config_slot_count((TpmsConfig)app->config));
        }
        slot = tpms_config_first_slot((TpmsConfig)app->config) + app->scan_step;
    } else {
        slot = (uint8_t)(app->config % TPMS_SLOT_COUNT);
    }

    if(tpms_session_retune(session, tpms_slot_frequency(slot), tpms_slot_modulation(slot))) {
        app->active_slot = slot;
    }
}

static void tpms_bridge_draw_callback(Canvas* canvas, void* context) {
    TpmsBridgeApp* app = context;
    furi_mutex_acquire(app->state_mutex, FuriWaitForever);
    tpms_view_draw(canvas, app);
    furi_mutex_release(app->state_mutex);
}

static void tpms_bridge_input_callback(InputEvent* event, void* context) {
    TpmsBridgeApp* app = context;
    furi_message_queue_put(app->input_queue, event, FuriWaitForever);
}

/** Lets the app be closed from the outside: `loader close`, `ufbt launch`.
 * Without this the system can only ask the user to press Back. */
static bool tpms_bridge_signal_callback(uint32_t signal, void* arg, void* context) {
    UNUSED(arg);
    TpmsBridgeApp* app = context;

    if(signal != FuriSignalExit) return false;

    /* From the list Back closes the app, from the detail screen it only
     * goes back. Switch to the list first so that the signal always
     * reaches its goal. */
    furi_mutex_acquire(app->state_mutex, FuriWaitForever);
    app->screen = TpmsScreenList;
    furi_mutex_release(app->state_mutex);

    const InputEvent event = {.type = InputTypeShort, .key = InputKeyBack};
    furi_message_queue_put(app->input_queue, &event, 0);
    return true;
}

typedef struct {
    TpmsBridgeApp* app;
    TpmsRawCapture* raw;
} TpmsLocalContext;

static void
    tpms_bridge_local_frame_callback(const TpmsFrame* frame, float rssi_dbm, void* context) {
    TpmsLocalContext* local = context;
    tpms_bridge_report_frame(local->app, frame, rssi_dbm);
    if(local->app->raw_save) tpms_raw_frame(frame, rssi_dbm, local->raw);
}

/** Local reception: runs only while the user has RX enabled and no other
 * screen owns the radio. */
static int32_t tpms_bridge_local_rx_thread(void* context) {
    TpmsBridgeApp* app = context;

    if(furi_mutex_acquire(app->radio_mutex, 0) != FuriStatusOk) {
        FURI_LOG_W(TAG, "radio busy, local rx cancelled");
        app->local_rx = false;
        app->rx_device_ready = false;
        view_port_update(app->view_port);
        return 0;
    }

    TpmsSession* session = tpms_session_alloc();
    TpmsRawCapture* raw = malloc(sizeof(TpmsRawCapture));
    memset(raw, 0, sizeof(TpmsRawCapture));
    TpmsLocalContext local_context = {.app = app, .raw = raw};
    tpms_session_set_frame_callback(session, tpms_bridge_local_frame_callback, &local_context);

    const uint8_t slot = tpms_config_is_auto((TpmsConfig)app->config) ?
                             tpms_config_first_slot((TpmsConfig)app->config) + app->scan_step :
                             (uint8_t)(app->config % TPMS_SLOT_COUNT);

    if(tpms_session_start(session, tpms_slot_frequency(slot), tpms_slot_modulation(slot))) {
        app->active_slot = slot;
        app->rx_device_external = tpms_session_is_external(session);
        app->rx_device_ready = true;
        app->radio_start_error = false;
        FURI_LOG_I(TAG, "RX started on %s", app->rx_device_external ? "EXT" : "INT");
        bool raw_was_enabled = false;
        uint8_t previous_slot = app->active_slot;
        if(app->auto_wake) {
            app->wake_next_tick = furi_get_tick() + furi_ms_to_ticks(TPMS_LF_PERIOD_MS);
        }

        while(app->rx_enabled && app->local_rx && !app->stop_requested && !app->radio_yield_requested) {
            const uint32_t now = furi_get_tick();
            if(!app->auto_wake) {
                app->wake_next_tick = 0;
            } else if(app->wake_next_tick == 0) {
                app->wake_next_tick = now + furi_ms_to_ticks(TPMS_LF_PERIOD_MS);
            }
            const bool wake_due = app->auto_wake &&
                                  (int32_t)(now - app->wake_next_tick) >= 0;

            if(wake_due || app->wake_requested) {
                app->wake_requested = false;
                app->wake_feedback_tick = now;
                app->wake_next_tick = now + furi_ms_to_ticks(TPMS_LF_PERIOD_MS);
                tpms_session_wake_pulse(session, TPMS_LF_PULSE_MS);
            }

            tpms_bridge_tune_radio(app, session);
            const bool enabled = app->raw_save;
            if(enabled != raw_was_enabled || app->active_slot != previous_slot) {
                tpms_raw_clear(raw);
                raw_was_enabled = enabled;
                previous_slot = app->active_slot;
                if(enabled) {
                    raw->frequency = tpms_slot_frequency(previous_slot);
                    raw->modulation = tpms_slot_modulation(previous_slot);
                    tpms_session_set_raw_callback(session, tpms_raw_interval, raw);
                } else {
                    tpms_session_set_raw_callback(session, NULL, NULL);
                }
            }
            /* The standard display callback is kept. Saving is triggered
             * only by a decoded valid frame, using the preceding pulse history. */
            tpms_session_pump(session, 100);
            if(enabled && raw->post_capture) tpms_raw_finish_post_capture(raw);
            if(enabled && raw->pending) tpms_raw_save(raw);
        }
        tpms_session_stop(session);
    } else {
        FURI_LOG_E(TAG, "cannot start local rx");
        app->radio_start_error = true;
    }

    tpms_session_free(session);
    free(raw);
    furi_mutex_release(app->radio_mutex);

    app->local_rx = false;
    app->rx_device_ready = false;
    app->rx_device_external = false;
    app->wake_next_tick = 0;
    app->last_radio_stop_tick = furi_get_tick();
    view_port_update(app->view_port);
    return 0;
}

static void tpms_bridge_request_local_rx_stop(TpmsBridgeApp* app) {
    app->local_rx = false;
    app->rx_device_ready = false;
    app->wake_next_tick = 0;
}

static void tpms_bridge_cleanup_local_rx(TpmsBridgeApp* app) {
    if(!app->local_thread) return;
    if(furi_thread_get_state(app->local_thread) != FuriThreadStateStopped) return;
    furi_thread_join(app->local_thread);
    furi_thread_free(app->local_thread);
    app->local_thread = NULL;
    app->last_radio_stop_tick = furi_get_tick();
}

static void tpms_bridge_stop_local_rx(TpmsBridgeApp* app) {
    tpms_bridge_request_local_rx_stop(app);
    if(!app->local_thread) return;

    furi_thread_join(app->local_thread);
    furi_thread_free(app->local_thread);
    app->local_thread = NULL;
    app->last_radio_stop_tick = furi_get_tick();
}

static void tpms_bridge_toggle_local_rx(TpmsBridgeApp* app) {
    app->rx_enabled = !app->rx_enabled;
    if(!app->rx_enabled) {
        app->local_rx = false;
        app->rx_device_ready = false;
        app->radio_start_error = false;
        app->wake_next_tick = 0;
        strlcpy(app->raw_message, "RX OFF", sizeof(app->raw_message));
    } else {
        app->radio_start_error = false;
        if(app->auto_wake) {
            app->wake_next_tick = furi_get_tick() + furi_ms_to_ticks(TPMS_LF_PERIOD_MS);
        }
        strlcpy(app->raw_message, "RX ON", sizeof(app->raw_message));
    }
}

static void tpms_bridge_stop_rx_from_ui(TpmsBridgeApp* app) {
    app->rx_enabled = false;
    app->radio_start_error = false;
    tpms_bridge_request_local_rx_stop(app);
    strlcpy(app->raw_message, "RX OFF", sizeof(app->raw_message));
}

/** Keeps local reception running while the radio is free, and gets it out
 * of the way once a USB session comes for the radio. */
static void tpms_bridge_reconcile_radio(TpmsBridgeApp* app) {
    tpms_bridge_cleanup_local_rx(app);

    if(app->local_thread && !app->local_rx) return;

    if(app->screen != TpmsScreenList) {
        tpms_bridge_request_local_rx_stop(app);
        return;
    }

    if(!app->rx_enabled) {
        tpms_bridge_request_local_rx_stop(app);
        return;
    }

    if(app->local_thread) return;
    if(app->stop_requested || app->radio_yield_requested || app->usb_streaming) return;

    /* Back off between attempts: if the radio refuses to start, do not
     * spin up the thread over and over. */
    const uint32_t now = furi_get_tick();
    if(app->last_radio_stop_tick != 0 &&
       now - app->last_radio_stop_tick < furi_ms_to_ticks(TPMS_RADIO_SWITCH_DELAY_MS)) {
        return;
    }
    if(app->radio_retry_tick != 0 &&
       now - app->radio_retry_tick < furi_ms_to_ticks(TPMS_RADIO_RETRY_MS)) {
        return;
    }
    app->radio_retry_tick = now;

    app->local_rx = true;
    app->rx_device_ready = false;
    app->local_thread =
        furi_thread_alloc_ex("TpmsLocalRx", 2048, tpms_bridge_local_rx_thread, app);
    furi_thread_start(app->local_thread);
}

/** Step through the radio configurations.
 *
 * The band and the modulation are one list rather than two settings: the
 * radio holds one of each at a time, and one ring of five entries is less
 * to remember than two separate cycles.
 */
static void tpms_bridge_step_config(TpmsBridgeApp* app, int8_t delta) {
    app->config = (uint8_t)((app->config + TpmsConfigCount + delta) % TpmsConfigCount);
    app->active_slot =
        tpms_config_is_auto((TpmsConfig)app->config) ?
            tpms_config_first_slot((TpmsConfig)app->config) :
            (uint8_t)app->config;
    app->scan_step = 0;
    app->scan_tick = 0;
}

static void tpms_bridge_select(TpmsBridgeApp* app, int8_t delta) {
    if(delta < 0) {
        if(app->selected > 0) app->selected--;
    } else if(app->store.count > 0 && app->selected + 1 < app->store.count) {
        app->selected++;
    }
    tpms_view_follow_selection(app);
}

static void tpms_bridge_toggle_auto_wake(TpmsBridgeApp* app) {
    app->auto_wake = !app->auto_wake;
    app->wake_requested = false;
    app->wake_feedback_tick = 0;
    app->wake_next_tick = app->auto_wake && app->local_rx ?
                              furi_get_tick() + furi_ms_to_ticks(TPMS_LF_PERIOD_MS) :
                              0;
}

static void tpms_bridge_apply_backlight(TpmsBridgeApp* app) {
    notification_message(
        app->notifications,
        app->keep_backlight_on ? &sequence_display_backlight_enforce_on :
                                 &sequence_display_backlight_enforce_auto);
}

static void tpms_bridge_raw_tx_stop_ex(TpmsBridgeApp* app, bool cooldown, bool keep_button_down) {
    const bool button_down = app->raw_tx_button_down;
    TpmsRawTx* tx = app->raw_tx;
    app->raw_tx = NULL;
    app->raw_tx_active = false;
    app->raw_tx_finishing = false;
    app->raw_tx_button_down = keep_button_down ? button_down : false;
    app->radio_yield_requested = false;
    subghz_block_generic_global.endless_tx = false;
    if(tx) tpms_raw_tx_free(tx);
    app->last_radio_stop_tick = furi_get_tick();
    app->tx_cooldown_until_tick =
        cooldown ? app->last_radio_stop_tick + furi_ms_to_ticks(TPMS_TX_COOLDOWN_MS) : 0;
    strlcpy(app->raw_message, cooldown ? "Wait..." : "Hold OK to send", sizeof(app->raw_message));
}

static void tpms_bridge_raw_tx_stop(TpmsBridgeApp* app) {
    tpms_bridge_raw_tx_stop_ex(app, true, false);
}

static void tpms_bridge_raw_tx_start(TpmsBridgeApp* app) {
    if(app->raw_tx_active) return;
    const uint32_t now = furi_get_tick();
    if(app->tx_cooldown_until_tick != 0 &&
       (int32_t)(now - app->tx_cooldown_until_tick) < 0) {
        app->raw_tx_button_down = false;
        strlcpy(app->raw_message, "Wait...", sizeof(app->raw_message));
        return;
    }

    char message[TPMS_RAW_MESSAGE_MAX];
    TpmsRawTx* tx = tpms_raw_tx_alloc(app->raw_tx_path, message, sizeof(message));
    strlcpy(app->raw_message, message, sizeof(app->raw_message));
    if(tx) {
        app->raw_tx = tx;
        app->raw_tx_active = true;
        app->raw_tx_finishing = false;
        app->raw_tx_button_down = true;
        app->tx_device_external = tx->device_external;
        app->raw_tx_display_until_tick =
            furi_get_tick() + furi_ms_to_ticks(TPMS_TX_DISPLAY_HOLD_MS);
        app->radio_yield_requested = true;
    } else {
        app->raw_tx_button_down = false;
    }
}

static void tpms_bridge_raw_tx_release(TpmsBridgeApp* app) {
    if(!app->raw_tx_active) return;
    app->raw_tx_button_down = false;
    subghz_block_generic_global.endless_tx = false;
    app->raw_tx_finishing = true;
    strlcpy(app->raw_message, "Finishing...", sizeof(app->raw_message));
}

static void tpms_bridge_handle_input(TpmsBridgeApp* app, const InputEvent* event, bool* running) {
    /* The wake pulse touches the hardware and may take almost a second —
     * do not hold the state lock across it. */
    furi_mutex_acquire(app->state_mutex, FuriWaitForever);

    const bool wake_screen = app->screen == TpmsScreenList || app->screen == TpmsScreenDetail;

    if(event->type == InputTypePress || event->type == InputTypeRelease) {
        if(app->screen == TpmsScreenRawTx && event->key == InputKeyOk) {
            if(event->type == InputTypePress) {
                app->raw_tx_button_down = true;
                furi_mutex_release(app->state_mutex);
                tpms_bridge_stop_local_rx(app);
                furi_delay_ms(TPMS_RADIO_SWITCH_DELAY_MS);
                furi_mutex_acquire(app->state_mutex, FuriWaitForever);
                tpms_bridge_raw_tx_start(app);
            } else {
                tpms_bridge_raw_tx_release(app);
            }
            furi_mutex_release(app->state_mutex);
            return;
        }
        furi_mutex_release(app->state_mutex);
        return;
    }

    if(event->type == InputTypeLong) {
        switch(event->key) {
        case InputKeyOk:
            if(app->screen == TpmsScreenList || app->screen == TpmsScreenDetail) {
                app->keep_backlight_on = !app->keep_backlight_on;
                tpms_bridge_apply_backlight(app);
            }
            break;

        case InputKeyLeft:
            app->raw_save = !app->raw_save;
            strlcpy(
                app->raw_message,
                app->raw_save ? "RAW save ON" : "RAW save OFF",
                sizeof(app->raw_message));
            break;

        case InputKeyUp:
        case InputKeyDown:
            /* Up and Down step the radio configuration, so picking a row
             * out of the list moves to holding them. */
            tpms_bridge_select(app, event->key == InputKeyUp ? -1 : 1);
            break;

        default:
            break;
        }

        furi_mutex_release(app->state_mutex);
        return;
    }

    if(event->type != InputTypeShort) {
        furi_mutex_release(app->state_mutex);
        return;
    }

    if(event->key == InputKeyLeft && wake_screen) {
        tpms_bridge_toggle_auto_wake(app);
        furi_mutex_release(app->state_mutex);
        return;
    }

    switch(event->key) {
    case InputKeyBack:
        if(app->screen == TpmsScreenDetail) {
            app->screen = TpmsScreenList;
        } else if(app->screen == TpmsScreenRawTx) {
            tpms_bridge_raw_tx_stop(app);
            app->screen = TpmsScreenRawFiles;
        } else if(
            app->screen == TpmsScreenRawMenu || app->screen == TpmsScreenRawOpen ||
            app->screen == TpmsScreenRawInfo ||
            app->screen == TpmsScreenRawConfirmDelete ||
            app->screen == TpmsScreenRawConfirmDeleteAll ||
            app->screen == TpmsScreenRawMessage || app->screen == TpmsScreenRawHelp) {
            app->screen = TpmsScreenRawFiles;
        } else if(app->screen == TpmsScreenRawFiles) {
            app->screen = TpmsScreenList;
        } else {
            *running = false;
        }
        break;

    case InputKeyOk:
        if(app->screen == TpmsScreenRawFiles) {
            if(app->raw_file_count > 0) {
                app->raw_menu_selected = 0;
                app->screen = TpmsScreenRawMenu;
            }
        } else if(app->screen == TpmsScreenRawMenu) {
            switch(app->raw_menu_selected) {
            case TpmsRawActionOpen:
                tpms_raw_decode_selected(app);
                app->screen = TpmsScreenRawOpen;
                break;
            case TpmsRawActionPlay:
                tpms_raw_build_path(
                    app->raw_tx_path,
                    sizeof(app->raw_tx_path),
                    app->raw_file_names[app->raw_file_selected]);
                strlcpy(app->raw_message, "Hold OK to send", sizeof(app->raw_message));
                app->screen = TpmsScreenRawTx;
                app->rx_enabled = false;
                furi_mutex_release(app->state_mutex);
                tpms_bridge_stop_local_rx(app);
                furi_delay_ms(TPMS_RADIO_SWITCH_DELAY_MS);
                furi_mutex_acquire(app->state_mutex, FuriWaitForever);
                break;
            case TpmsRawActionRawInfo:
                tpms_raw_update_selected_info(app);
                app->screen = TpmsScreenRawInfo;
                break;
            case TpmsRawActionRename: {
                furi_mutex_release(app->state_mutex);
                const bool renamed = tpms_raw_rename_selected(app);
                furi_mutex_acquire(app->state_mutex, FuriWaitForever);
                app->screen = renamed ? TpmsScreenRawFiles : TpmsScreenRawMessage;
                break;
            }
            case TpmsRawActionDelete:
                app->screen = TpmsScreenRawConfirmDelete;
                break;
            case TpmsRawActionDeleteAll:
                app->screen = TpmsScreenRawConfirmDeleteAll;
                break;
            default:
                break;
            }
        } else if(app->screen == TpmsScreenRawConfirmDelete) {
            tpms_raw_delete_selected(app);
            app->screen = TpmsScreenRawFiles;
        } else if(app->screen == TpmsScreenRawConfirmDeleteAll) {
            tpms_raw_delete_all(app);
            app->screen = TpmsScreenRawFiles;
        } else if(app->screen == TpmsScreenRawTx) {
        } else if(
            app->screen == TpmsScreenRawOpen || app->screen == TpmsScreenRawInfo ||
            app->screen == TpmsScreenRawMessage) {
            app->screen = TpmsScreenRawFiles;
        } else if(app->screen == TpmsScreenList) {
            if(app->store.count > 0) {
                tpms_bridge_stop_rx_from_ui(app);
                tpms_view_follow_selection(app);
                app->screen = TpmsScreenDetail;
            } else {
                tpms_bridge_toggle_local_rx(app);
            }
        } else if(app->screen == TpmsScreenDetail) {
            app->screen = TpmsScreenList;
        } else {
            app->screen = TpmsScreenList;
        }
        break;

    case InputKeyLeft:
        if(app->screen == TpmsScreenDetail) {
            app->screen = TpmsScreenList;
        } else if(app->screen == TpmsScreenRawTx) {
            tpms_bridge_raw_tx_stop(app);
            app->screen = TpmsScreenRawFiles;
        } else if(
            app->screen == TpmsScreenRawMenu || app->screen == TpmsScreenRawOpen ||
            app->screen == TpmsScreenRawInfo ||
            app->screen == TpmsScreenRawConfirmDelete ||
            app->screen == TpmsScreenRawConfirmDeleteAll ||
            app->screen == TpmsScreenRawMessage || app->screen == TpmsScreenRawHelp) {
            app->screen = TpmsScreenRawFiles;
        } else if(app->screen == TpmsScreenRawFiles) {
            app->screen = TpmsScreenList;
        } else if(app->screen == TpmsScreenList && app->store.count > 0) {
            tpms_bridge_stop_rx_from_ui(app);
            tpms_bridge_select(app, -1);
        } else {
            tpms_bridge_toggle_auto_wake(app);
        }
        break;

    case InputKeyRight:
        if(app->screen == TpmsScreenList || app->screen == TpmsScreenDetail) {
            tpms_raw_refresh_files(app);
            app->rx_enabled = false;
            app->screen = TpmsScreenRawFiles;
            furi_mutex_release(app->state_mutex);
            tpms_bridge_stop_local_rx(app);
            furi_mutex_acquire(app->state_mutex, FuriWaitForever);
        } else if(app->screen == TpmsScreenRawFiles) {
            app->raw_help_page = 0;
            app->screen = TpmsScreenRawHelp;
        }
        break;

    case InputKeyUp:
    case InputKeyDown: {
        const int8_t delta = event->key == InputKeyUp ? -1 : 1;
        /* On the list these step the band and the modulation; on the
         * detail screen they walk from one sensor to the next. */
        if(app->screen == TpmsScreenRawFiles) {
            tpms_raw_select_file(app, delta);
        } else if(app->screen == TpmsScreenRawMenu) {
            tpms_raw_select_action(app, delta);
        } else if(app->screen == TpmsScreenRawHelp) {
            app->raw_help_page =
                (uint8_t)((app->raw_help_page + TPMS_RAW_HELP_PAGES + delta) % TPMS_RAW_HELP_PAGES);
        } else if(app->screen == TpmsScreenDetail) {
            tpms_bridge_select(app, delta);
        } else if(app->screen == TpmsScreenList && app->store.count > 0) {
            tpms_bridge_stop_rx_from_ui(app);
            tpms_bridge_select(app, delta);
        } else {
            tpms_bridge_step_config(app, delta);
        }
        break;
    }

    default:
        break;
    }

    furi_mutex_release(app->state_mutex);
}

static TpmsBridgeApp* tpms_bridge_app_alloc(void) {
    TpmsBridgeApp* app = malloc(sizeof(TpmsBridgeApp));
    memset(app, 0, sizeof(TpmsBridgeApp));

    app->state_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->radio_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->input_queue = furi_message_queue_alloc(TPMS_INPUT_QUEUE_SIZE, sizeof(InputEvent));
    subghz_devices_init();

    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, tpms_bridge_draw_callback, app);
    view_port_input_callback_set(app->view_port, tpms_bridge_input_callback, app);

    app->gui = furi_record_open(RECORD_GUI);
    app->notifications = furi_record_open(RECORD_NOTIFICATION);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);

    app->config = TpmsConfigAuto433;

    furi_thread_set_signal_callback(furi_thread_get_current(), tpms_bridge_signal_callback, app);

    app->cli_registry = furi_record_open(RECORD_CLI);
    /* Set the stack explicitly: the command formats strings and holds a
     * FuriString, and the default CLI command stack is rather small. */
    cli_registry_add_command_ex(
        app->cli_registry,
        TPMS_CLI_COMMAND_NAME,
        CliCommandFlagParallelSafe,
        tpms_cli_command,
        app,
        TPMS_CLI_STACK_SIZE);

    return app;
}

static void tpms_bridge_tick_notifications(TpmsBridgeApp* app) {
    const uint32_t now = furi_get_tick();

    if(app->screen == TpmsScreenList && app->rx_device_ready &&
       now - app->last_rx_blink_tick > furi_ms_to_ticks(1000)) {
        app->last_rx_blink_tick = now;
        notification_message(app->notifications, &sequence_blink_cyan_10);
    }

    if(app->screen == TpmsScreenRawTx && app->raw_tx_active) {
        TpmsRawTx* tx = app->raw_tx;
        if(tx && !subghz_devices_is_async_complete_tx(tx->device)) {
            if(now - app->last_tx_blink_tick > furi_ms_to_ticks(250)) {
                app->last_tx_blink_tick = now;
                notification_message(app->notifications, &sequence_blink_magenta_10);
            }
        } else {
            furi_mutex_acquire(app->state_mutex, FuriWaitForever);
            if(app->raw_tx_button_down && !app->raw_tx_finishing && app->screen == TpmsScreenRawTx) {
                tpms_bridge_raw_tx_stop_ex(app, false, true);
                tpms_bridge_raw_tx_start(app);
            } else {
                tpms_bridge_raw_tx_stop(app);
                if(app->tx_cooldown_until_tick == 0) {
                    strlcpy(app->raw_message, "Hold OK to send", sizeof(app->raw_message));
                }
            }
            furi_mutex_release(app->state_mutex);
        }
    }

    if(app->tx_cooldown_until_tick != 0 &&
       (int32_t)(now - app->tx_cooldown_until_tick) >= 0) {
        furi_mutex_acquire(app->state_mutex, FuriWaitForever);
        app->tx_cooldown_until_tick = 0;
        if(app->screen == TpmsScreenRawTx && !app->raw_tx_active) {
            strlcpy(app->raw_message, "Hold OK to send", sizeof(app->raw_message));
        }
        furi_mutex_release(app->state_mutex);
    }
}

static void tpms_bridge_app_free(TpmsBridgeApp* app) {
    cli_registry_delete_command(app->cli_registry, TPMS_CLI_COMMAND_NAME);
    furi_record_close(RECORD_CLI);

    notification_message(app->notifications, &sequence_reset_rgb);
    notification_message(app->notifications, &sequence_display_backlight_enforce_auto);
    furi_record_close(RECORD_NOTIFICATION);

    gui_remove_view_port(app->gui, app->view_port);
    furi_record_close(RECORD_GUI);
    view_port_free(app->view_port);

    furi_message_queue_free(app->input_queue);
    subghz_devices_deinit();
    furi_mutex_free(app->radio_mutex);
    furi_mutex_free(app->state_mutex);
    free(app);
}

int32_t tpms_bridge_app(void* p) {
    UNUSED(p);
    TpmsBridgeApp* app = tpms_bridge_app_alloc();

    bool running = true;
    InputEvent event;
    while(running) {
        if(furi_message_queue_get(app->input_queue, &event, 200) == FuriStatusOk) {
            tpms_bridge_handle_input(app, &event, &running);
        }

        tpms_bridge_reconcile_radio(app);
        tpms_bridge_tick_notifications(app);
        view_port_update(app->view_port);
    }

    tpms_bridge_raw_tx_stop(app);
    tpms_bridge_stop_local_rx(app);

    /* While a CLI command is running its code lives in this .fap, so the
     * app must not be unloaded. Ask the session to finish and wait.
     *
     * Wait in bounded steps: if the session does not respond for some
     * reason, staying on screen with a clear message beats pulling the
     * code out from under a running thread. */
    app->stop_requested = true;
    uint32_t waited_ms = 0;
    while(app->cli_sessions > 0) {
        furi_delay_ms(20);
        waited_ms += 20;
        if(waited_ms > TPMS_CLI_STOP_TIMEOUT_MS) {
            FURI_LOG_E(TAG, "cli session did not stop, keeping app loaded");
            furi_mutex_acquire(app->state_mutex, FuriWaitForever);
            app->exit_blocked = true;
            furi_mutex_release(app->state_mutex);
            view_port_update(app->view_port);
            waited_ms = 0;
        }
    }

    tpms_bridge_app_free(app);
    return 0;
}
