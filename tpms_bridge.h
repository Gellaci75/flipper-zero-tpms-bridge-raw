#pragma once

#include "tpms_radio.h"
#include "tpms_session.h"
#include "tpms_store.h"

#include <furi.h>
#include <cli/cli.h>
#include <gui/gui.h>
#include <gui/view_port.h>
#include <input/input.h>
#include <notification/notification.h>

#define TPMS_CLI_COMMAND_NAME "tpms_rx"
#define TPMS_APP_TITLE "TPMS Studio"
#define TPMS_APP_VERSION "v1.11"

/** Stack size of the CLI command thread. */
#define TPMS_CLI_STACK_SIZE (4 * 1024)

/** How long to wait for the CLI session to finish on exit, ms. */
#define TPMS_CLI_STOP_TIMEOUT_MS 3000

/** How long the CLI command waits for local RX to release the radio. */
#define TPMS_RADIO_YIELD_TIMEOUT_MS 2000

/** Application screen. */
typedef enum {
    TpmsScreenList, /**< sensor list */
    TpmsScreenDetail, /**< details of the selected sensor */
    TpmsScreenRawFiles, /**< saved raw captures */
    TpmsScreenRawMenu, /**< actions for selected raw file */
    TpmsScreenRawOpen, /**< decoded TPMS data from selected raw file */
    TpmsScreenRawInfo, /**< selected raw file info */
    TpmsScreenRawConfirmDelete, /**< delete confirmation */
    TpmsScreenRawConfirmDeleteAll, /**< delete all confirmation */
    TpmsScreenRawMessage, /**< short placeholder/status message */
    TpmsScreenRawHelp, /**< built-in usage notes */
    TpmsScreenRawTx, /**< hold-to-transmit status */
} TpmsScreen;

#define TPMS_RAW_FILE_ROWS 4
#define TPMS_RAW_FILE_MAX 16
#define TPMS_RAW_NAME_MAX 64
#define TPMS_RAW_MESSAGE_MAX 48
#define TPMS_RAW_RENAME_MAX 64

/** Shared state: what gets drawn on the screen, plus arbitration of the
 * radio between local reception and the USB session. */
typedef struct {
    FuriMutex* state_mutex;

    /* The radio is owned either by local RX or by the CLI session. */
    FuriMutex* radio_mutex;

    /* While a CLI command is running the app must not be unloaded: the
     * command code lives in this very .fap. */
    volatile uint32_t cli_sessions;
    volatile bool stop_requested;

    /* The CLI session asks local RX to release the radio and to keep off
     * it while the flag is raised. */
    volatile bool radio_yield_requested;

    volatile bool local_rx; /**< the local RX thread should be running */
    bool rx_enabled; /**< user-controlled local RX start/stop */
    uint32_t radio_retry_tick; /**< when the radio was last claimed */
    bool usb_streaming;
    bool exit_blocked;
    volatile bool wake_requested; /**< single pulse requested by a key */
    volatile bool raw_save; /**< hold Left to toggle */
    volatile bool auto_wake; /**< periodic pulses */
    bool keep_backlight_on;
    volatile bool rx_device_ready;
    volatile bool radio_start_error;
    bool rx_device_external;
    bool tx_device_external;
    bool raw_tx_active;
    bool raw_tx_finishing;
    bool raw_tx_button_down;
    uint32_t raw_tx_display_until_tick;
    uint32_t wake_feedback_tick;
    uint32_t wake_next_tick;
    uint32_t last_rx_blink_tick;
    uint32_t last_tx_blink_tick;
    uint32_t tx_cooldown_until_tick;
    uint32_t last_radio_stop_tick;

    /* Radio configuration, changed from the keys and applied by whoever
     * owns the radio. */
    volatile uint8_t config; /**< TpmsConfig */
    volatile uint8_t active_slot; /**< the combination on air right now */
    uint8_t scan_step; /**< which combination the scan is on */
    uint32_t scan_tick; /**< when the scan last stepped */

    TpmsStore store;
    TpmsScreen screen;
    uint8_t selected;
    uint8_t scroll;
    uint8_t raw_file_count;
    uint8_t raw_file_selected;
    uint8_t raw_file_scroll;
    uint8_t raw_menu_selected;
    uint8_t raw_help_page;
    uint64_t raw_file_size;
    uint32_t raw_file_frequency;
    char raw_file_protocol[16];
    bool raw_file_tpms_valid;
    uint8_t raw_file_tpms_protocol;
    uint32_t raw_file_tpms_id;
    int32_t raw_file_tpms_pressure_kpa_x100;
    int16_t raw_file_tpms_temperature_c;
    int16_t raw_file_tpms_rssi_x10;
    uint8_t raw_file_tpms_have;
    uint8_t raw_file_tpms_flags;
    char raw_tx_path[96];
    void* raw_tx;
    char raw_file_names[TPMS_RAW_FILE_MAX][TPMS_RAW_NAME_MAX];
    char raw_message[TPMS_RAW_MESSAGE_MAX];
    char rename_buffer[TPMS_RAW_RENAME_MAX];

    Gui* gui;
    NotificationApp* notifications;
    ViewPort* view_port;
    FuriMessageQueue* input_queue;
    CliRegistry* cli_registry;
    FuriThread* local_thread;
} TpmsBridgeApp;

/** Store a received frame in the shared state (for the screen). */
void tpms_bridge_report_frame(TpmsBridgeApp* app, const TpmsFrame* frame, float rssi_dbm);

/** Keep the radio in step with the settings, and step the scan along.
 * Called from whichever loop owns the radio. */
void tpms_bridge_tune_radio(TpmsBridgeApp* app, TpmsSession* session);

/** Implementation of the tpms_rx CLI command. */
void tpms_cli_command(PipeSide* pipe, FuriString* args, void* context);
