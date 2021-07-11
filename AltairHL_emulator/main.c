/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

// Hardware definition
#include "hw/azure_sphere_learning_path.h"

// DevX Libraries
#include "dx_config.h"
#include "dx_exit_codes.h"
#include "dx_gpio.h"
#include "dx_terminate.h"
#include "dx_timer.h"
#include "dx_intercore.h"
#include "dx_version.h"
#include "../IntercoreContract/intercore_contract.h"

// System Libraries
#include "applibs_versions.h"
#include <applibs/log.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <applibs/storage.h>

#include "curldefs.h"
#include "weather.h"
#include "utils.h"
#include "comms_manager_wolf.h"
#include "front_panel_virtual.h"
#include "iotc_manager.h"

#include "intel8080.h"
#include "88dcdd.h"
#include "sphere_panel.h"
#include "memory.h"

#define ALTAIR_ON_AZURE_SPHERE_VERSION "3.0"

#ifdef ALTAIR_FRONT_PANEL_CLICK
#include "front_panel_click.h"
#endif

#ifdef ALTAIR_FRONT_PANEL_RETRO_CLICK
#include "front_panel_retro_click.h"
#endif

#ifdef ALTAIR_FRONT_PANEL_KIT
#include "front_panel_kit.h"
#endif // ALTAIR_FRONT_PANEL_KIT

#ifdef ALTAIR_FRONT_PANEL_NONE
#include "front_panel_none.h"
#endif // ALTAIR_FRONT_PANEL_NONE

#define Log_Debug(f_, ...) dx_Log_Debug((f_), ##__VA_ARGS__)

#define NETWORK_INTERFACE "wlan0"
#define DX_LOGGING_ENABLED FALSE

// https://docs.microsoft.com/en-us/azure/iot-pnp/overview-iot-plug-and-play
#define IOT_PLUG_AND_PLAY_MODEL_ID "dtmi:com:example:azuresphere:altair;1"
#define CORE_ENVIRONMENT_COMPONENT_ID "2e319eae-7be5-4a0c-ba47-9353aa6ca96a"
#define CORE_DISK_CACHE_COMPONENT_ID "9b684af8-21b9-42aa-91e4-621d5428e497"
#define BASIC_SAMPLES_DIRECTORY "BasicSamples"

static bool loadRomImage(char *romImageName, uint16_t loadAddress);
static void *altair_thread(void *arg);
static void connection_status_led_off_handler(EventLoopTimer *eventLoopTimer);
static void connection_status_led_on_handler(EventLoopTimer *eventLoopTimer);
static void device_twin_set_temperature_handler(DX_DEVICE_TWIN_BINDING *deviceTwinBinding);
static void intercore_disk_cache_receive_msg_handler(void *data_block, ssize_t message_length);
static void intercore_environment_receive_msg_handler(void *data_block, ssize_t message_length);
static void measure_sensor_handler(EventLoopTimer *eventLoopTimer);
static void mqtt_dowork_handler(EventLoopTimer *eventLoopTimer);
static void panel_refresh_handler(EventLoopTimer *eventLoopTimer);
static void process_control_panel_commands(void);
static void WatchdogMonitorTimerHandler(EventLoopTimer *eventLoopTimer);

DX_USER_CONFIG userConfig;
static float Temperature = 0.0; // storage for weather data
static const char *AltairMsg = "\x1b[2J\r\nAzure Sphere - Altair 8800 Emulator\r\n";

char msgBuffer[MSG_BUFFER_BYTES] = {0};

// CPU CPU_RUNNING STATE (CPU_STOPPED/CPU_RUNNING)
CPU_OPERATING_MODE cpu_operating_mode = CPU_STOPPED;

static intel8080_t cpu;
uint8_t memory[64 * 1024]; // Altair system memory.

ALTAIR_COMMAND cmd_switches;
uint16_t bus_switches = 0x00;

int altair_spi_fd = -1;
int console_fd = -1;

const struct itimerspec watchdogInterval = {{60, 0}, {60, 0}};
timer_t watchdogTimer;

const uint8_t reverse_lut[16] = {0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf};

// basic app load helpers.
static bool haveCtrlPending = false;
static char haveCtrlCharacter = 0x00;
static bool haveAppLoad = false;
static int basicAppLength = 0;
static int appLoadPtr = 0;
static uint8_t *ptrBasicApp = NULL;

static bool haveTerminalInputMessage = false;
static bool haveTerminalOutputMessage = false;
static int terminalInputMessageLen = 0;
static int terminalOutputMessageLen = 0;
static int altairInputBufReadIndex = 0;
static int altairOutputBufReadIndex = 0;

static pthread_cond_t wait_message_processed_cond = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t wait_message_processed_mutex = PTHREAD_MUTEX_INITIALIZER;
static char *input_data = NULL;

bool local_serial = true;
bool dirty_buffer = false;
bool send_messages = false;
bool invoke_mqtt_sync = false;
bool renderText = false;

static char Log_Debug_Time_buffer[64];

INTERCORE_ENVIRONMENT_DATA_BLOCK_T intercore_send_block;
INTERCORE_ENVIRONMENT_DATA_BLOCK_T intercore_recv_block;
INTERCORE_ENVIRONMENT_T current_environment;

INTERCORE_DISK_DATA_BLOCK_T intercore_disk_block;

DX_INTERCORE_BINDING intercore_environment_ctx = {.sockFd = -1,
                                                  .nonblocking_io = true,
                                                  .rtAppComponentId = CORE_ENVIRONMENT_COMPONENT_ID,
                                                  .interCoreCallback = intercore_environment_receive_msg_handler,
                                                  .intercore_recv_block = &intercore_recv_block,
                                                  .intercore_recv_block_length = sizeof(intercore_recv_block)};

DX_INTERCORE_BINDING intercore_disk_cache_ctx = {.sockFd = -1,
                                                 .nonblocking_io = true,
                                                 .rtAppComponentId = CORE_DISK_CACHE_COMPONENT_ID,
                                                 .interCoreCallback = intercore_disk_cache_receive_msg_handler,
                                                 .intercore_recv_block = &intercore_disk_block,
                                                 .intercore_recv_block_length = sizeof(intercore_disk_block)};

#ifdef ALTAIR_FRONT_PANEL_CLICK

CLICK_4X4_BUTTON_MODE click_4x4_key_mode = CONTROL_MODE;

matrix8x8_t panel8x8 = {.interfaceId = MT3620_ISU1_SPI, .chipSelectId = MT3620_SPI_CS_B, .busSpeed = 10000000, .handle = -1, .bitmap = {0}};

key4x4_t key4x4 = {.interfaceId = MT3620_ISU1_SPI, .chipSelectId = MT3620_SPI_CS_A, .busSpeed = 10000000, .handle = -1, .bitmap = 0, .debouncePeriodMilliseconds = 500};

DX_GPIO_BINDING buttonB = {.pin = BUTTON_B, .direction = DX_INPUT, .detect = DX_GPIO_DETECT_LOW, .name = "buttonB"};

// turn off notifications
DX_TIMER_BINDING turnOffNotificationsTimer = {.period = {0, 0}, .name = "turnOffNotificationsTimer", .handler = turn_off_notifications_handler};

#endif //  ALTAIR_FRONT_PANEL_CLICK

#ifdef ALTAIR_FRONT_PANEL_RETRO_CLICK

CLICK_4X4_BUTTON_MODE click_4x4_key_mode = CONTROL_MODE;
DX_GPIO_BINDING buttonB = {.pin = BUTTON_B, .direction = DX_INPUT, .detect = DX_GPIO_DETECT_LOW, .name = "buttonB"};
as1115_t retro_click = {.interfaceId = ISU2, .handle = -1, .bitmap64 = 0, .keymap = 0, .debouncePeriodMilliseconds = 500};

// turn off notifications
DX_TIMER_BINDING turnOffNotificationsTimer = {.period = {0, 0}, .name = "turnOffNotificationsTimer", .handler = turn_off_notifications_handler};

#endif //  ALTAIR_FRONT_PANEL_RETRO_CLICK

#ifdef ALTAIR_FRONT_PANEL_KIT

// static DX_GPIO memoryCS = { .pin = MEMORY_CS, .direction = DX_OUTPUT, .initialState =
// GPIO_Value_High, .invertPin = false, .name = "memory CS" }; static DX_GPIO sdCS = { .pin = SD_CS,
// .direction = DX_OUTPUT, .initialState = GPIO_Value_High, .invertPin = false, .name = "SD_CS" };

DX_GPIO_BINDING switches_chip_select = {.pin = SWITCHES_CHIP_SELECT, .direction = DX_OUTPUT, .initialState = GPIO_Value_High, .invertPin = false, .name = "switches CS"};

DX_GPIO_BINDING switches_load = {.pin = SWITCHES_LOAD, .direction = DX_OUTPUT, .initialState = GPIO_Value_High, .invertPin = false, .name = "switchs Load"};

DX_GPIO_BINDING led_store = {.pin = LED_STORE, .direction = DX_OUTPUT, .initialState = GPIO_Value_High, .invertPin = false, .name = "LED store"};

static DX_GPIO_BINDING led_master_reset = {.pin = LED_MASTER_RESET, .direction = DX_OUTPUT, .initialState = GPIO_Value_High, .invertPin = false, .name = "LED master reset"};

static DX_GPIO_BINDING led_output_enable = {
    .pin = LED_OUTPUT_ENABLE, .direction = DX_OUTPUT, .initialState = GPIO_Value_Low, .invertPin = false, .name = "LED output enable"}; // set OE initial state low

#endif // ALTAIR_FRONT_PANEL_KIT

static DX_GPIO_BINDING buttonA = {.pin = BUTTON_A, .direction = DX_INPUT, .detect = DX_GPIO_DETECT_LOW, .name = "buttonA"};
static DX_GPIO_BINDING azure_iot_connected_led = {
    .pin = AZURE_CONNECTED_LED, .direction = DX_OUTPUT, .initialState = GPIO_Value_Low, .invertPin = true, .name = "azure_iot_connected_led"};

// Common Timers
DX_TIMER_BINDING restartDeviceOneShotTimer = {.period = {0, 0}, .name = "restartDeviceOneShotTimer", .handler = delay_restart_device_handler};
static DX_TIMER_BINDING connectionStatusLedOffTimer = {.period = {0, 0}, .name = "connectionStatusLedOffTimer", .handler = connection_status_led_off_handler};
static DX_TIMER_BINDING connectionStatusLedOnTimer = {.period = {0, 0}, .name = "connectionStatusLedOnTimer", .handler = connection_status_led_on_handler};
static DX_TIMER_BINDING measure_sensor_timer = {.period = {5, 0}, .name = "measure_sensor_timer", .handler = measure_sensor_handler};
static DX_TIMER_BINDING memory_diagnostics_timer = {.period = {60, 0}, .name = "memory_diagnostics_timer", .handler = memory_diagnostics_handler};
static DX_TIMER_BINDING mqtt_do_work_timer = {.period = {0, 300 * OneMS}, .name = "mqtt_do_work_timer", .handler = mqtt_dowork_handler};
static DX_TIMER_BINDING panel_refresh_timer = {.period = {0, 20 * OneMS}, .name = "panel_refresh_timer", .handler = panel_refresh_handler};
static DX_TIMER_BINDING watchdogMonitorTimer = {.period = {5, 0}, .name = "watchdogMonitorTimer", .handler = WatchdogMonitorTimerHandler};

// Azure IoT Central Properties (Device Twins)
DX_DEVICE_TWIN_BINDING dt_channelId = {.twinProperty = "DesiredChannelId", .twinType = DX_TYPE_INT, .handler = device_twin_set_channel_id_handler};
DX_DEVICE_TWIN_BINDING dt_diskCacheHits = {.twinProperty = "DiskCacheHits", .twinType = DX_TYPE_INT};
DX_DEVICE_TWIN_BINDING dt_diskCacheMisses = {.twinProperty = "DiskCacheMisses", .twinType = DX_TYPE_INT};
DX_DEVICE_TWIN_BINDING dt_diskTotalErrors = {.twinProperty = "DiskTotalErrors", .twinType = DX_TYPE_INT};
DX_DEVICE_TWIN_BINDING dt_diskTotalWrites = {.twinProperty = "DiskTotalWrites", .twinType = DX_TYPE_INT};
static DX_DEVICE_TWIN_BINDING dt_desiredCpuState = {.twinProperty = "DesiredCpuState", .twinType = DX_TYPE_BOOL, .handler = device_twin_set_cpu_state_handler};
static DX_DEVICE_TWIN_BINDING dt_desiredLedBrightness = {.twinProperty = "DesiredLedBrightness", .twinType = DX_TYPE_INT, .handler = device_twin_set_led_brightness_handler};
static DX_DEVICE_TWIN_BINDING dt_desiredLocalSerial = {.twinProperty = "DesiredLocalSerial", .twinType = DX_TYPE_BOOL, .handler = device_twin_set_local_serial_handler};
static DX_DEVICE_TWIN_BINDING dt_desiredTemperature = {.twinProperty = "DesiredTemperature", .twinType = DX_TYPE_INT, .handler = device_twin_set_temperature_handler};
static DX_DEVICE_TWIN_BINDING dt_reportedDeviceStartTime = {.twinProperty = "ReportedDeviceStartTime", .twinType = DX_TYPE_STRING};
static DX_DEVICE_TWIN_BINDING dt_reportedTemperature = {.twinProperty = "ReportedTemperature", .twinType = DX_TYPE_INT};
static DX_DEVICE_TWIN_BINDING dt_softwareVersion = {.twinProperty = "SoftwareVersion", .twinType = DX_TYPE_STRING};

// Azure IoT Central Commands (Direct Methods)
static DX_DIRECT_METHOD_BINDING dm_restartDevice = {.methodName = "RestartDevice", .handler = RestartDeviceHandler};

// Initialize Sets
static DX_GPIO_BINDING *gpioSet[] = {&azure_iot_connected_led,
                                     &buttonA
#if defined(ALTAIR_FRONT_PANEL_CLICK) || defined(ALTAIR_FRONT_PANEL_RETRO_CLICK)
                                     ,
                                     &buttonB
#endif // ALTAIR_FRONT_PANEL_CLICK

#ifdef ALTAIR_FRONT_PANEL_KIT
                                     ,
                                     &switches_load,
                                     &switches_chip_select,
                                     &led_master_reset,
                                     &led_store,
                                     &led_output_enable
//&memoryCS, &sdCS
#endif // ALTAIR_FRONT_PANEL_KIT
};

static DX_TIMER_BINDING *timerSet[] = {&connectionStatusLedOnTimer,
                                       &connectionStatusLedOffTimer,
                                       &memory_diagnostics_timer,
                                       &measure_sensor_timer,
                                       &restartDeviceOneShotTimer,
                                       &mqtt_do_work_timer,
                                       &panel_refresh_timer,
                                       &watchdogMonitorTimer
#if defined(ALTAIR_FRONT_PANEL_CLICK) || defined(ALTAIR_FRONT_PANEL_RETRO_CLICK)
                                       ,
                                       &turnOffNotificationsTimer
#endif // ALTAIR_FRONT_PANEL_CLICK
};

static DX_DEVICE_TWIN_BINDING *deviceTwinBindingSet[] = {&dt_reportedDeviceStartTime, &dt_channelId,          &dt_desiredCpuState,     &dt_desiredLedBrightness,
                                                         &dt_desiredLocalSerial,      &dt_desiredTemperature, &dt_reportedTemperature, &dt_diskCacheHits,
                                                         &dt_diskCacheMisses,         &dt_diskTotalWrites,    &dt_diskTotalErrors};

DX_DIRECT_METHOD_BINDING *directMethodBindingSet[] = {&dm_restartDevice};

// End of variable declarations

/// <summary>
/// This timer extends the app level lease watchdog timer
/// </summary>
/// <param name="eventLoopTimer"></param>
static void WatchdogMonitorTimerHandler(EventLoopTimer *eventLoopTimer)
{
    if (ConsumeEventLoopTimerEvent(eventLoopTimer) != 0) {
        dx_terminate(DX_ExitCode_ConsumeEventLoopTimeEvent);
        return;
    }
    timer_settime(watchdogTimer, 0, &watchdogInterval, NULL);
}

/// <summary>
/// Set up watchdog timer - the lease is extended via the WatchdogMonitorTimerHandler function
/// </summary>
/// <param name=""></param>
void SetupWatchdog(void)
{
    struct sigevent alarmEvent;
    alarmEvent.sigev_notify = SIGEV_SIGNAL;
    alarmEvent.sigev_signo = SIGALRM;
    alarmEvent.sigev_value.sival_ptr = &watchdogTimer;

    if (timer_create(CLOCK_MONOTONIC, &alarmEvent, &watchdogTimer) == 0) {
        if (timer_settime(watchdogTimer, 0, &watchdogInterval, NULL) == -1) {
            Log_Debug("Issue setting watchdog timer. %s %d\n", strerror(errno), errno);
        }
    }
}

static void mqtt_connected_cb(void)
{
    static bool connection_initialised = false;
    static const char *connected_message = "\r\nCONNECTED TO AZURE SPHERE ALTAIR 8800 EMULATOR VERSION: %s, DevX VERSION: %s.\r\n\r\n";
    static const char *reconnected_message =
        "\r\nRECONNECTED TO AZURE SPHERE ALTAIR 8800 EMULATOR VERSION: %s, DevX VERSION: "
        "%s.\r\n\r\n";

    if (!connection_initialised) {
        connection_initialised = true;
        int len = snprintf(msgBuffer, sizeof(msgBuffer), connected_message, ALTAIR_ON_AZURE_SPHERE_VERSION, AZURE_SPHERE_DEVX_VERSION);
        queue_mqtt_message(msgBuffer, (size_t)len);
        cpu_operating_mode = CPU_RUNNING;
        // if (dt_desiredCpuState.twinState) {
        //	cpu_operating_mode = CPU_RUNNING;
        //}
    } else {
        int len = snprintf(msgBuffer, sizeof(msgBuffer), reconnected_message, ALTAIR_ON_AZURE_SPHERE_VERSION, AZURE_SPHERE_DEVX_VERSION);
        queue_mqtt_message(msgBuffer, (size_t)len);
    }
}

static bool load_application(const char *fileName)
{
    char filePathAndName[50];
    snprintf(filePathAndName, sizeof(filePathAndName), "%s/%s", BASIC_SAMPLES_DIRECTORY, fileName);

    Log_Debug("LOADING '%s'\n", fileName);
    int GameFd = Storage_OpenFileInImagePackage(filePathAndName);
    if (GameFd >= 0) {
        // get length.
        off_t length = lseek(GameFd, 0, SEEK_END);
        ptrBasicApp = (uint8_t *)malloc((size_t)length + 9);
        memset((void *)ptrBasicApp, 0x00, (size_t)length + 9);
        lseek(GameFd, 0, SEEK_SET);
        read(GameFd, ptrBasicApp + 5, (size_t)length);
        close(GameFd);

        // wrap loaded app with 2 leading <CR><LF>, and 2 trailing <CR><LF>

        ptrBasicApp[0] = 0x0d;
        ptrBasicApp[1] = 0x0a;
        ptrBasicApp[2] = 0x0d;
        ptrBasicApp[3] = 0x0a;
        ptrBasicApp[length + 3] = 0x0d;
        ptrBasicApp[length + 4] = 0x0a;
        ptrBasicApp[length + 5] = 0x0d;
        ptrBasicApp[length + 6] = 0x0a;

        terminalInputMessageLen = (int)(length + 4);
        terminalOutputMessageLen = (int)(length + 4);

        appLoadPtr = 0;
        basicAppLength = (int)(length + 9); // add extra <CR><LF> * 2
        haveAppLoad = true;
    } else {
        return false;
    }
    return true;
}

/// <summary>
/// Callback handler for Inter-Core Messaging
/// </summary>
static void intercore_environment_receive_msg_handler(void *data_block, ssize_t message_length)
{
    INTERCORE_ENVIRONMENT_DATA_BLOCK_T *block = (INTERCORE_ENVIRONMENT_DATA_BLOCK_T *)data_block;

    switch (block->ic_msg_type) {
    case ALTAIR_IC_ENVIRONMENT:
        current_environment.temperature = block->environment.temperature;
        current_environment.pressure = block->environment.pressure;
        break;
    default:
        break;
    }
}

static void intercore_disk_cache_receive_msg_handler(void *data_block, ssize_t message_length)
{
    vdisk_cache_response_cb((INTERCORE_DISK_DATA_BLOCK_T *)data_block);
}

/// <summary>
/// Device Twin Handler to set the desired temperature value
/// </summary>
static void device_twin_set_temperature_handler(DX_DEVICE_TWIN_BINDING *deviceTwinBinding)
{
    // validate data is sensible range before applying
    if (deviceTwinBinding->twinType == DX_TYPE_INT && *(int *)deviceTwinBinding->twinState >= -20 && *(int *)deviceTwinBinding->twinState <= 80) {
        // Send the desired temperate to the real-time core enviromon app
        intercore_send_block.ic_msg_type = ALTAIR_IC_THERMOSTAT;
        intercore_send_block.environment.desired_temperature = *(int *)deviceTwinBinding->twinState;
        dx_intercorePublish(&intercore_environment_ctx, &intercore_send_block, sizeof(INTERCORE_ENVIRONMENT_T));
        // acknowledge the device twin
        dx_deviceTwinAckDesiredState(deviceTwinBinding, deviceTwinBinding->twinState, DX_DEVICE_TWIN_COMPLETED);
    } else {
        dx_deviceTwinAckDesiredState(deviceTwinBinding, deviceTwinBinding->twinState, DX_DEVICE_TWIN_ERROR);
    }
}

/// <summary>
/// Read sensor and send to Azure IoT
/// </summary>
static void measure_sensor_handler(EventLoopTimer *eventLoopTimer)
{
    if (ConsumeEventLoopTimerEvent(eventLoopTimer) != 0) {
        dx_terminate(DX_ExitCode_ConsumeEventLoopTimeEvent);
        return;
    }
    static uint8_t device_twin_update_rate = 0;
    int current_temperature = (int)current_environment.temperature;

    // Send request to RT Core for current environment data
    intercore_send_block.ic_msg_type = ALTAIR_IC_ENVIRONMENT;
    dx_intercorePublish(&intercore_environment_ctx, &intercore_send_block, sizeof(INTERCORE_ENVIRONMENT_T));

    if (++device_twin_update_rate > 5) { // send every 6 updates = every 30 seconds
        device_twin_update_rate = 0;
        dx_deviceTwinReportState(&dt_diskCacheHits, dt_diskCacheHits.twinState);
        dx_deviceTwinReportState(&dt_diskCacheMisses, dt_diskCacheMisses.twinState);
        dx_deviceTwinReportState(&dt_diskTotalErrors, dt_diskTotalErrors.twinState);
        dx_deviceTwinReportState(&dt_diskTotalWrites, dt_diskTotalWrites.twinState);
        dx_deviceTwinReportState(&dt_reportedTemperature, &current_temperature);
    }
}

/// <summary>
/// Handle inbound MQTT messages
/// </summary>
/// <param name="topic_name"></param>
/// <param name="topic_name_size"></param>
/// <param name="message"></param>
/// <param name="message_size"></param>
static void handle_inbound_message(const char *topic_name, size_t topic_name_size, const char *message, size_t message_size)
{
    char command[30];
    char *data;
    bool send_cr = false;
    size_t application_message_size;

    TOPIC_TYPE topic = topic_type((char *)topic_name, topic_name_size);
    data = (char *)message;
    application_message_size = message_size;

    switch (topic) {
    case TOPIC_DATA_SUB: // data message
        // upper case incoming message
        memset(command, 0, sizeof(command));

        if (application_message_size > 0 && data[application_message_size - 1] == '\r') { // is last char carriage return ?
            send_cr = true;
            application_message_size--;
        }

        for (int i = 0; i < sizeof(command) - 1 && i < application_message_size; i++) { // -1 to allow for trailing null
            command[i] = (char)toupper(data[i]);
        }

        // if command is load then try looking for in baked in samples otherwise pass on to Altair
        // emulator
        if (strncmp(command, "LOAD ", 5) == 0 && application_message_size > 5 && (command[application_message_size - 1] == '"')) {
            command[application_message_size - 1] = 0x00; // replace the '"' with \0
            if (load_application(&command[6])) {
                return;
            }
        }

        switch (cpu_operating_mode) {
        case CPU_RUNNING:
            if (application_message_size > 0) { // for example just cr was send so don't try send chars to CPU
                input_data = data;

                altairInputBufReadIndex = 0;
                altairOutputBufReadIndex = 0;
                terminalInputMessageLen = (int)application_message_size;
                terminalOutputMessageLen = (int)application_message_size;

                haveTerminalInputMessage = true;
                haveTerminalOutputMessage = true;

                pthread_mutex_lock(&wait_message_processed_mutex);
                pthread_cond_wait(&wait_message_processed_cond, &wait_message_processed_mutex);
                pthread_mutex_unlock(&wait_message_processed_mutex);
            }

            if (send_cr) {
                haveCtrlCharacter = 0x0d;
                haveCtrlPending = true;

                pthread_mutex_lock(&wait_message_processed_mutex);
                pthread_cond_wait(&wait_message_processed_cond, &wait_message_processed_mutex);
                pthread_mutex_unlock(&wait_message_processed_mutex);
            }
            break;
        case CPU_STOPPED:
            process_virtual_input(command, process_control_panel_commands);
            break;
        default:
            break;
        }
        break;
    case TOPIC_PASTE_SUB: // paste message
        break;
    case TOPIC_CONTROL_SUB: // control message
        if (data[0] >= 'A' && data[0] <= 'Z') {
            if (data[0] == 'M') { // CPU Monitor mode
                cpu_operating_mode = cpu_operating_mode == CPU_RUNNING ? CPU_STOPPED : CPU_RUNNING;
                if (cpu_operating_mode == CPU_STOPPED) {
                    queue_mqtt_message("\r\nCPU MONITOR> ", 15);
                    // publish_message("\r\nCPU MONITOR> ", 15, pub_topic_data);
                } else {
                    queue_mqtt_message("\r\n", 2);
                    // publish_message("\r\n", 2, pub_topic_data);
                }
            } else {
                haveCtrlPending = true;
                haveCtrlCharacter = data[0] & 31; // https://en.wikipedia.org/wiki/Control_character

                pthread_mutex_lock(&wait_message_processed_mutex);
                pthread_cond_wait(&wait_message_processed_cond, &wait_message_processed_mutex);
                pthread_mutex_unlock(&wait_message_processed_mutex);
            }
        }
        break;
    case TOPIC_VDISK_SUB: // vdisk response
        vdisk_mqtt_response_cb(data);
        break;
    default:
        break;
    }
}

static void publish_callback_wolf(MqttMessage *msg)
{
    handle_inbound_message(msg->topic_name, msg->topic_name_len, msg->buffer, msg->buffer_len);
}

static void connection_status_led_off_handler(EventLoopTimer *eventLoopTimer)
{
    if (ConsumeEventLoopTimerEvent(eventLoopTimer) != 0) {
        dx_terminate(DX_ExitCode_ConsumeEventLoopTimeEvent);
        return;
    }
    dx_gpioOff(&azure_iot_connected_led);
}

/// <summary>
/// Flash LEDs timer handler
/// </summary>
static void connection_status_led_on_handler(EventLoopTimer *eventLoopTimer)
{
    static int init_sequence = 25;
    static bool firstConnect = true;

    if (ConsumeEventLoopTimerEvent(eventLoopTimer) != 0) {
        dx_terminate(DX_ExitCode_ConsumeEventLoopTimeEvent);
        return;
    }

    if (init_sequence-- > 0) {

        dx_gpioOn(&azure_iot_connected_led);
        // on for 100ms off for 100ms = 200 ms in total
        dx_timerOneShotSet(&connectionStatusLedOnTimer, &(struct timespec){0, 200 * OneMS});
        dx_timerOneShotSet(&connectionStatusLedOffTimer, &(struct timespec){0, 100 * OneMS});

    } else if (dx_isAzureConnected() && is_mqtt_connected()) {

        dx_gpioOn(&azure_iot_connected_led);
        // on for 5000ms off for 200ms = 5200 ms in total
        dx_timerOneShotSet(&connectionStatusLedOnTimer, &(struct timespec){5, 0});
        dx_timerOneShotSet(&connectionStatusLedOffTimer, &(struct timespec){4, 800 * OneMS});

        // Update device start time device twin
        if (firstConnect && dx_isAzureConnected()) {

            // Update SoftwareVersion Device Twin
            snprintf(msgBuffer, sizeof(msgBuffer), "Altair on Sphere version: %s, DevX version: %s", ALTAIR_ON_AZURE_SPHERE_VERSION, AZURE_SPHERE_DEVX_VERSION);
            dx_deviceTwinReportState(&dt_softwareVersion, msgBuffer);

            dx_deviceTwinReportState(&dt_reportedDeviceStartTime, dx_getCurrentUtc(msgBuffer, sizeof(msgBuffer))); // DX_TYPE_STRING
            firstConnect = false;
        }

    } else if (dx_isNetworkReady()) {

        dx_gpioOn(&azure_iot_connected_led);
        // on for 100ms off for 1300ms = 1400 ms in total
        dx_timerOneShotSet(&connectionStatusLedOnTimer, &(struct timespec){1, 400 * OneMS});
        dx_timerOneShotSet(&connectionStatusLedOffTimer, &(struct timespec){0, 700 * OneMS});

    } else {

        dx_gpioOn(&azure_iot_connected_led);
        // on for 700ms off for 700ms = 1400 ms in total
        dx_timerOneShotSet(&connectionStatusLedOnTimer, &(struct timespec){1, 400 * OneMS});
        dx_timerOneShotSet(&connectionStatusLedOffTimer, &(struct timespec){0, 100 * OneMS});
    }
}

static void mqtt_dowork_handler(EventLoopTimer *eventLoopTimer)
{
    if (ConsumeEventLoopTimerEvent(eventLoopTimer) != 0) {
        dx_terminate(DX_ExitCode_ConsumeEventLoopTimeEvent);
        return;
    }

    if (dirty_buffer) {
        send_messages = true;
    }
}

static uint8_t sphere_port_in(uint8_t port)
{
    static bool readingTempData = false;
    static char data[10];
    static int readPtr = 0;
    uint8_t retVal = 0;

    if (port == 42) {
        if (!readingTempData) {
            readPtr = 0;
            snprintf(data, 10, "%3.2f", Temperature);
            readingTempData = true;
        }

        retVal = data[readPtr++];
        if (retVal == 0x00) {
            readingTempData = false;
        }
    }

    if (port == 43) {
        if (!readingTempData) {
            readPtr = 0;
            snprintf(data, 10, "%3.2f", current_environment.temperature);
            readingTempData = true;
        }

        retVal = data[readPtr++];
        if (retVal == 0x00) {
            readingTempData = false;
        }
    }

    if (port == 44) {
        if (!readingTempData) {
            readPtr = 0;
            snprintf(data, 10, "%4.1f ", current_environment.pressure);
            readingTempData = true;
        }

        retVal = data[readPtr++];
        if (retVal == 0x00) {
            readingTempData = false;
        }
    }
    return retVal;
}

static void sphere_port_out(uint8_t port, uint8_t data)
{
    struct location_info *locData;
    char *weatherData;

    // get IP and Weather data.
    if (port == 32 && data == 1) {
        locData = GetLocationData();
        weatherData = GetCurrentWeather(locData, &Temperature);
    }

    // publish the telemetry to IoTC
    if (port == 32 && data == 2) {
        publish_telemetry(Temperature);
    }
}

static char altair_read_terminal(void)
{
    uint8_t rxBuffer[2] = {0};
    char retVal;

    if (haveCtrlPending) {
        haveCtrlPending = false;

        pthread_mutex_lock(&wait_message_processed_mutex);
        pthread_cond_signal(&wait_message_processed_cond);
        pthread_mutex_unlock(&wait_message_processed_mutex);

        return haveCtrlCharacter;
    }

    if (console_fd != -1) {
        ssize_t iRead = read(console_fd, rxBuffer, 1);
        if (iRead > 0) {
            // Log_Debug("Rx: 0x%02x (%c)\n", rxBuffer[0], rxBuffer[0] >= 0x20 ? rxBuffer[0] : '.');
            return (rxBuffer[0]);
        }
    }

    if (haveTerminalInputMessage) {
        // if (altairInputBufReadIndex > 0) {
        retVal = input_data[altairInputBufReadIndex++];
        //}

        if (altairInputBufReadIndex >= terminalInputMessageLen) {
            haveTerminalInputMessage = false;

            pthread_mutex_lock(&wait_message_processed_mutex);
            pthread_cond_signal(&wait_message_processed_cond);
            pthread_mutex_unlock(&wait_message_processed_mutex);
        }
        return retVal;

    } else if (haveAppLoad) {
        retVal = ptrBasicApp[appLoadPtr++];
        publish_character(retVal);

        if (appLoadPtr == basicAppLength) {
            haveAppLoad = false;
            free(ptrBasicApp);
            appLoadPtr = 0;
        }
        return retVal;
    }

    return 0;
}

static void altair_write_terminal(char c)
{
    if (c > 0x80)
        c -= 0x80;

    if (haveTerminalOutputMessage) {
        altairOutputBufReadIndex++;

        if (altairOutputBufReadIndex > terminalOutputMessageLen)
            haveTerminalOutputMessage = false;
    }

    if (!haveTerminalOutputMessage && !haveAppLoad) {
        publish_character(c);
    }
}

void process_control_panel_commands(void)
{
    if (cpu_operating_mode == CPU_STOPPED) {
        switch (cmd_switches) {
        case RUN_CMD:
            cpu_operating_mode = CPU_RUNNING;
            break;
        case STOP_CMD:
            i8080_examine(&cpu, cpu.registers.pc);
            break;
        case SINGLE_STEP:
            i8080_cycle(&cpu);
            publish_cpu_state("Single step", cpu.address_bus, cpu.data_bus);
            break;
        case EXAMINE:
            i8080_examine(&cpu, bus_switches);
            publish_cpu_state("Examine", cpu.address_bus, cpu.data_bus);
            break;
        case EXAMINE_NEXT:
            i8080_examine_next(&cpu);
            publish_cpu_state("Examine next", cpu.address_bus, cpu.data_bus);
            break;
        case DEPOSIT:
            i8080_deposit(&cpu, (uint8_t)(bus_switches & 0xff));
            publish_cpu_state("Deposit", cpu.address_bus, cpu.data_bus);
            break;
        case DEPOSIT_NEXT:
            i8080_deposit_next(&cpu, (uint8_t)(bus_switches & 0xff));
            publish_cpu_state("Deposit next", cpu.address_bus, cpu.data_bus);
            break;
        default:
            break;
        }
    }

    if (cmd_switches & STOP_CMD) {
        cpu_operating_mode = CPU_STOPPED;
    }
    cmd_switches = 0x00;
}

static void update_panel_leds(uint8_t status, uint8_t data, uint16_t bus)
{
    status = reverse_lut[(status & 0xf0) >> 4] | reverse_lut[status & 0xf] << 4;
    data = reverse_lut[(data & 0xf0) >> 4] | reverse_lut[data & 0xf] << 4;
    bus = reverse_lut[(bus & 0xf000) >> 12] << 8 | reverse_lut[(bus & 0x0f00) >> 8] << 12 | reverse_lut[(bus & 0xf0) >> 4] | reverse_lut[bus & 0xf] << 4;

    update_panel_status_leds(status, data, bus);
}

static void read_panel_input(void)
{
    static GPIO_Value_Type buttonAState = GPIO_Value_High;

    if (dx_gpioStateGet(&buttonA, &buttonAState)) { // Button A shortcut to start CPM on the device
        cpu_operating_mode = CPU_STOPPED;
        process_control_panel_commands();
        bus_switches = 0xff00;
        cmd_switches = EXAMINE;
        process_control_panel_commands();
        cpu_operating_mode = CPU_RUNNING;
        process_control_panel_commands();
    }
    read_altair_panel_switches(process_control_panel_commands);
}

static void panel_refresh_handler(EventLoopTimer *eventLoopTimer)
{
    if (ConsumeEventLoopTimerEvent(eventLoopTimer) != 0) {
        dx_terminate(DX_ExitCode_ConsumeEventLoopTimeEvent);
        return;
    }
    update_panel_leds(cpu.cpuStatus, cpu.data_bus, cpu.address_bus);
    read_panel_input();
}

static inline uint8_t sense(void)
{
    return bus_switches >> 8;
}

static bool loadRomImage(char *romImageName, uint16_t loadAddress)
{
    int romFd = Storage_OpenFileInImagePackage(romImageName);
    if (romFd == -1)
        return false;

    off_t length = lseek(romFd, 0, SEEK_END);
    lseek(romFd, 0, SEEK_SET);
    read(romFd, &memory[loadAddress], (size_t)length);
    close(romFd);

    return true;
}

static void load8kRom(void)
{
    const uint8_t rom[] = {
#include "Altair8800/8krom.h"
    };
    memcpy(memory, rom, sizeof(rom));
}

static void print_console_banner(void)
{
    for (int x = 0; x < strlen(AltairMsg); x++) {
        altair_write_terminal(AltairMsg[x]);
    }
}

#pragma GCC push_options
#pragma GCC optimize("O0")
static void *altair_thread(void *arg)
{
    Log_Debug("Altair Thread starting...\n");
    print_console_banner();

    memset(memory, 0x00, 64 * 1024); // clear memory.

    // initially no disk controller.
    disk_controller_t disk_controller;
    disk_controller.disk_function = disk_function;
    disk_controller.disk_select = disk_select;
    disk_controller.disk_status = disk_status;
    disk_controller.read = disk_read;
    disk_controller.write = disk_write;
    disk_controller.sector = sector;

    disk_drive.disk1.fp = Storage_OpenFileInImagePackage("Disks/cpm63k.dsk");
    if (disk_drive.disk1.fp == -1) {
        Log_Debug("Failed to load CPM Disk\n");
    }

    // drive 2 is virtual (Python Server or MQTT Server).
    disk_drive.disk2.fp = -1;
    disk_drive.disk2.diskPointer = 0;
    disk_drive.disk2.sector = 0;
    disk_drive.disk2.track = 0;

    i8080_reset(&cpu, (port_in)altair_read_terminal, (port_out)altair_write_terminal, sense, &disk_controller, (azure_sphere_port_in)sphere_port_in,
                (azure_sphere_port_out)sphere_port_out);
    load8kRom(); // load 8k rom basic into memory at address 0x0000.

    // load Disk Loader at 0xff00
    if (!loadRomImage("Disks/88dskrom.bin", 0xff00))
        Log_Debug("Failed to load Disk ROM image\n");

    i8080_examine(&cpu, 0x0000); // 0xff00 loads from disk, 0x0000 loads basic

    while (1) {
        if (cpu_operating_mode == CPU_RUNNING) {
            i8080_cycle(&cpu);
        }

        if (send_messages) {
            if (dirty_buffer) {
                send_partial_message();
            }
            dirty_buffer = send_messages = false;
        }
    }

    return NULL;
}
#pragma GCC pop_options

/// <summary>
///  Initialize PeripheralGpios, device twins, direct methods, timers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static void InitPeripheralAndHandlers(void)
{
    dx_Log_Debug_Init(Log_Debug_Time_buffer, sizeof(Log_Debug_Time_buffer));
    curl_global_init(CURL_GLOBAL_DEFAULT);
    dx_gpioSetOpen(gpioSet, NELEMS(gpioSet));
    init_altair_hardware();

#ifndef ALTAIR_FRONT_PANEL_NONE
    // dx_startThreadDetached(panel_thread, NULL, "panel_thread");
#endif // !ALTAIR_FRONT_PANEL_NONE

    dx_azureConnect(&userConfig, NETWORK_INTERFACE, IOT_PLUG_AND_PLAY_MODEL_ID);
    init_mqtt(publish_callback_wolf, mqtt_connected_cb);
    dx_deviceTwinSubscribe(deviceTwinBindingSet, NELEMS(deviceTwinBindingSet));
    dx_timerSetStart(timerSet, NELEMS(timerSet));
    dx_directMethodSubscribe(directMethodBindingSet, NELEMS(directMethodBindingSet));

    dx_intercoreConnect(&intercore_environment_ctx);
    dx_intercoreConnect(&intercore_disk_cache_ctx);

    dx_timerOneShotSet(&connectionStatusLedOnTimer, &(struct timespec){1, 400 * OneMS});
    dx_startThreadDetached(altair_thread, NULL, "altair_thread");

    SetupWatchdog();
}

/// <summary>
///     Close PeripheralGpios and handlers.
/// </summary>
static void ClosePeripheralAndHandlers(void)
{
    dx_azureToDeviceStop();
    dx_deviceTwinUnsubscribe();
    dx_directMethodUnsubscribe();
    dx_timerEventLoopStop();
    dx_gpioSetClose(gpioSet, NELEMS(gpioSet));
    curl_global_cleanup();
}

int main(int argc, char *argv[])
{
    dx_registerTerminationHandler();
    if (!dx_configParseCmdLineArguments(argc, argv, &userConfig)) {
        return dx_getTerminationExitCode();
    }
    InitPeripheralAndHandlers();

    // Main loop
    while (!dx_isTerminationRequired()) {
        int result = EventLoop_Run(dx_timerGetEventLoop(), -1, true);
        // Continue if interrupted by signal, e.g. due to breakpoint being set.
        if (result == -1 && errno != EINTR) {
            dx_terminate(DX_ExitCode_Main_EventLoopFail);
        }
    }
    ClosePeripheralAndHandlers();
    Log_Debug("\n\nApplication exiting. Last known exit code: %d\n", dx_getTerminationExitCode());
    return dx_getTerminationExitCode();
}