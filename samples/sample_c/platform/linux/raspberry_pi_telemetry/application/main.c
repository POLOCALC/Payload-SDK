/**
 * psdk_telemetry - DJI Payload SDK telemetry bridge for POLOCALC (Matrice 400 through the E-Port).
 *
 * Connects to the aircraft, subscribes flight-controller telemetry topics and streams every received sample as a
 * framed binary record (see telemetry_stream.h) to stdout, or to a file with --output. It is meant to be started
 * by lager (UAVs/DJI_M400), which reads the stream, decodes it and logs it, like it does for the Matrice 600.
 * Human-readable logs go to stderr. Stops cleanly on SIGINT/SIGTERM.
 *
 * Usage: psdk_telemetry [--output FILE|-] [--imu HZ] [--fast HZ] [--status HZ] [--slow HZ] [--stats S]
 *                       [--high-power 24|17|13.6|off] [--serial SN] [--print] [--verbose]
 */
#include <dji_platform.h>
#include <dji_logger.h>
#include <dji_core.h>
#include <dji_aircraft_info.h>
#include <dji_fc_subscription.h>
#include <dji_power_management.h>

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "osal/osal.h"
#include "osal/osal_fs.h"
#include "hal_uart.h"
#include "hal_i2c.h"
#include "dji_sdk_app_info.h"
#include "telemetry_stream.h"
#include "service.h"

#define PROGRAM_NAME          "psdk_telemetry"
#define DEFAULT_SERIAL_NUMBER "PSDK12345678XX"   /* same as the DJI demo, so an activation done with it stays valid */
#define LATEST_MAX_SIZE       128             /* largest topic: ESC data, 80 bytes */
#define SUBSCRIPTION_MAX_BYTES_PER_FREQ 242      /* PSDK limit, see DjiFcSubscription_SubscribeTopic() */
#define LINK_TIMEOUT_S        10                 /* no fast-group data for this long: the link is lost, exit */

/* ---------------------------------------------------------------------------------------------- topic table */
/* Topics are subscribed in up to four groups, one rate each (the PSDK allows at most 4 rates, 242 bytes per rate).
   The link to the aircraft runs at 115200 baud (about 11.5 kB/s): at the defaults the telemetry takes about 3.5 kB/s.
   With everything at higher rates (~10 kB/s) the aircraft stops answering after about a minute and drops the link. */
typedef enum {
    GROUP_IMU = 0,          /* --imu (off by default): IMU navigation data, which the aircraft only gives at 50 Hz */
    GROUP_FAST,             /* --fast:   attitude, IMU, altitudes, GPS/RTK and fused position, obstacles, RC sticks */
    GROUP_STATUS,           /* --status: visual odometry, RC link, GPS quality, flight state and anomalies, ESCs */
    GROUP_SLOW,             /* --slow:   batteries, GPS date/time, home point */
    GROUP_COUNT
} E_Group;

static const char *const s_groupNames[GROUP_COUNT] = {"imu", "fast", "status", "slow"};

typedef struct {
    E_DjiFcSubscriptionTopic       topic;
    const char                    *name;        /* used by the decoders (lager), max 31 characters */
    uint16_t                       size;
    E_Group                        group;
    DjiReceiveDataOfTopicCallback  callback;
    /* runtime */
    E_DjiDataSubscriptionTopicFreq frequency;
    bool                           subscribed;
    uint32_t                       received;
    uint8_t                        latest[LATEST_MAX_SIZE];
    bool                           haveLatest;
} T_Topic;

static T_DjiReturnCode OnTopicData(int index, const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts);

/* the SDK callback doesn't say which topic it is for: one small callback per table entry */
#define DEFINE_CALLBACK(i) \
    static T_DjiReturnCode Callback##i(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) \
    { return OnTopicData(i, data, size, ts); }
DEFINE_CALLBACK(0)  DEFINE_CALLBACK(1)  DEFINE_CALLBACK(2)  DEFINE_CALLBACK(3)  DEFINE_CALLBACK(4)
DEFINE_CALLBACK(5)  DEFINE_CALLBACK(6)  DEFINE_CALLBACK(7)  DEFINE_CALLBACK(8)  DEFINE_CALLBACK(9)
DEFINE_CALLBACK(10) DEFINE_CALLBACK(11) DEFINE_CALLBACK(12) DEFINE_CALLBACK(13) DEFINE_CALLBACK(14)
DEFINE_CALLBACK(15) DEFINE_CALLBACK(16) DEFINE_CALLBACK(17) DEFINE_CALLBACK(18) DEFINE_CALLBACK(19)
DEFINE_CALLBACK(20) DEFINE_CALLBACK(21) DEFINE_CALLBACK(22) DEFINE_CALLBACK(23) DEFINE_CALLBACK(24)
DEFINE_CALLBACK(25) DEFINE_CALLBACK(26) DEFINE_CALLBACK(27) DEFINE_CALLBACK(28) DEFINE_CALLBACK(29)
DEFINE_CALLBACK(30) DEFINE_CALLBACK(31) DEFINE_CALLBACK(32) DEFINE_CALLBACK(33) DEFINE_CALLBACK(34)
DEFINE_CALLBACK(35) DEFINE_CALLBACK(36) DEFINE_CALLBACK(37) DEFINE_CALLBACK(38) DEFINE_CALLBACK(39)

#define TOPIC(id, name, type, group, n) {DJI_FC_SUBSCRIPTION_TOPIC_##id, name, sizeof(type), group, Callback##n}

/* Every flight-controller topic of the PSDK that the Matrice 400 provides. Left out:
   - rejected by the Matrice 400 (error 0x00000100): BATTERY_INFO (whole battery; BATTERY_SINGLE_INFO_INDEX1/2 give
     each battery), GIMBAL_STATUS, THREE_GIMBAL_DATA, GPS_CONTROL_LEVEL, STATUS_LANDINGGEAR, GIMBAL_CONTROL_MODE,
     HEIGHT_RELATIVE, HARD_SYNC,
   - DJI gimbals on the aircraft's gimbal ports, none on POLOCALC: GIMBAL_ANGLES (50 Hz only), GIMBAL_ANGLES_ON_POS_NO1..7
     (NO1 also has the same topic id as IMU_ATTI_NAVI_DATA_WITH_TIMESTAMP in dji_fc_subscription.h).
   A topic the aircraft doesn't support is reported as a warning and skipped. */
static T_Topic s_topics[] = {
    /* imu: 64 bytes, 50 Hz only */
    TOPIC(IMU_ATTI_NAVI_DATA_WITH_TIMESTAMP, "imu_atti_navi", T_DjiFcSubscriptionImuAttiNaviDataWithTimestamp, GROUP_IMU, 0),
    /* fast: 226 bytes */
    TOPIC(QUATERNION,            "quaternion",            T_DjiFcSubscriptionQuaternion,            GROUP_FAST,   1),
    TOPIC(VELOCITY,              "velocity",              T_DjiFcSubscriptionVelocity,              GROUP_FAST,   2),
    TOPIC(ACCELERATION_GROUND,   "acceleration_ground",   T_DjiFcSubscriptionAccelerationGround,    GROUP_FAST,   3),
    TOPIC(ACCELERATION_BODY,     "acceleration_body",     T_DjiFcSubscriptionAccelerationBody,      GROUP_FAST,   4),
    TOPIC(ACCELERATION_RAW,      "acceleration_raw",      T_DjiFcSubscriptionAccelerationRaw,       GROUP_FAST,   5),
    TOPIC(ANGULAR_RATE_FUSIONED, "angular_rate_fused",    T_DjiFcSubscriptionAngularRateFusioned,   GROUP_FAST,   6),
    TOPIC(ANGULAR_RATE_RAW,      "angular_rate_raw",      T_DjiFcSubscriptionAngularRateRaw,        GROUP_FAST,   7),
    TOPIC(COMPASS,               "compass",               T_DjiFcSubscriptionCompass,               GROUP_FAST,   8),
    TOPIC(ALTITUDE_FUSED,        "altitude_fused",        T_DjiFcSubscriptionAltitudeFused,         GROUP_FAST,   9),
    TOPIC(ALTITUDE_BAROMETER,    "altitude_barometer",    T_DjiFcSubscriptionAltitudeBarometer,     GROUP_FAST,   10),
    TOPIC(HEIGHT_FUSION,         "height_fusion",         T_DjiFcSubscriptionHeightFusion,          GROUP_FAST,   11),
    TOPIC(GPS_POSITION,          "gps_position",          T_DjiFcSubscriptionGpsPosition,           GROUP_FAST,   12),
    TOPIC(GPS_VELOCITY,          "gps_velocity",          T_DjiFcSubscriptionGpsVelocity,           GROUP_FAST,   13),
    TOPIC(RTK_POSITION,          "rtk_position",          T_DjiFcSubscriptionRtkPosition,           GROUP_FAST,   14),
    TOPIC(RTK_VELOCITY,          "rtk_velocity",          T_DjiFcSubscriptionRtkVelocity,           GROUP_FAST,   15),
    TOPIC(RTK_YAW,               "rtk_yaw",               T_DjiFcSubscriptionRtkYaw,                GROUP_FAST,   16),
    TOPIC(RTK_POSITION_INFO,     "rtk_position_info",     T_DjiFcSubscriptionRtkPositionInfo,       GROUP_FAST,   17),
    TOPIC(RTK_YAW_INFO,          "rtk_yaw_info",          T_DjiFcSubscriptionRtkYawInfo,            GROUP_FAST,   18),
    TOPIC(POSITION_FUSED,        "position_fused",        T_DjiFcSubscriptionPositionFused,         GROUP_FAST,   19),
    TOPIC(AVOID_DATA,            "avoid_data",            T_DjiFcSubscriptionAvoidData,             GROUP_FAST,   20),
    TOPIC(RC,                    "rc",                    T_DjiFcSubscriptionRC,                    GROUP_FAST,   21),
    /* status: 159 bytes */
    TOPIC(POSITION_VO,           "position_vo",           T_DjiFcSubscriptionPositionVO,            GROUP_STATUS, 22),
    TOPIC(RC_WITH_FLAG_DATA,     "rc_with_flag",          T_DjiFcSubscriptionRCWithFlagData,        GROUP_STATUS, 23),
    TOPIC(GPS_DETAILS,           "gps_details",           T_DjiFcSubscriptionGpsDetails,            GROUP_STATUS, 24),
    TOPIC(GPS_SIGNAL_LEVEL,      "gps_signal_level",      T_DjiFcSubscriptionGpsSignalLevel,        GROUP_STATUS, 25),
    TOPIC(STATUS_FLIGHT,         "flight_status",         T_DjiFcSubscriptionFlightStatus,          GROUP_STATUS, 26),
    TOPIC(STATUS_DISPLAYMODE,    "display_mode",          T_DjiFcSubscriptionDisplaymode,           GROUP_STATUS, 27),
    TOPIC(STATUS_MOTOR_START_ERROR, "motor_start_error",  T_DjiFcSubscriptionMotorStartError,       GROUP_STATUS, 28),
    TOPIC(CONTROL_DEVICE,        "control_device",        T_DjiFcSubscriptionControlDevice,         GROUP_STATUS, 29),
    TOPIC(RTK_CONNECT_STATUS,    "rtk_connect_status",    T_DjiFcSubscriptionRTKConnectStatus,      GROUP_STATUS, 30),
    TOPIC(FLIGHT_ANOMALY,        "flight_anomaly",        T_DjiFcSubscriptionFlightAnomaly,         GROUP_STATUS, 31),
    TOPIC(ESC_DATA,              "esc_data",              T_DjiFcSubscriptionEscData,               GROUP_STATUS, 32),
    /* slow: 95 bytes */
    TOPIC(BATTERY_SINGLE_INFO_INDEX1, "battery1",         T_DjiFcSubscriptionSingleBatteryInfo,     GROUP_SLOW,   33),
    TOPIC(BATTERY_SINGLE_INFO_INDEX2, "battery2",         T_DjiFcSubscriptionSingleBatteryInfo,     GROUP_SLOW,   34),
    TOPIC(GPS_DATE,              "gps_date",              T_DjiFcSubscriptionGpsDate,               GROUP_SLOW,   35),
    TOPIC(GPS_TIME,              "gps_time",              T_DjiFcSubscriptionGpsTime,               GROUP_SLOW,   36),
    TOPIC(ALTITUDE_OF_HOMEPOINT, "altitude_of_homepoint", T_DjiFcSubscriptionAltitudeOfHomePoint,   GROUP_SLOW,   37),
    TOPIC(HOME_POINT_SET_STATUS, "home_point_set_status", T_DjiFcSubscriptionHomePointSetStatus,    GROUP_SLOW,   38),
    TOPIC(HOME_POINT_INFO,       "home_point_info",       T_DjiFcSubscriptionHomePointInfo,         GROUP_SLOW,   39),
};
#define TOPIC_COUNT ((int) (sizeof(s_topics) / sizeof(s_topics[0])))

/* the TOPIC_TABLE record must fit in one stream record */
_Static_assert(2 + (sizeof(s_topics) / sizeof(s_topics[0])) * (7 + TELEMETRY_STREAM_NAME_SIZE) <= TELEMETRY_STREAM_MAX_PAYLOAD,
               "topic table larger than TELEMETRY_STREAM_MAX_PAYLOAD");

/* ---------------------------------------------------------------------------------------------- options/state */
static struct {
    const char *output;
    int         groupHz[GROUP_COUNT];       /* --imu, --fast, --status, --slow; 0 = group not subscribed */
    int         statsS;
    const char *serial;
    bool        print;
    bool        verbose;
    const char *highPower;                  /* --high-power: E-Port VCC voltage to request, or "off" */
    bool        service;                    /* --service: sockets and RC widgets instead of stdout (service.h) */
    T_ServiceOptions serviceOptions;
} s_opt = {"-", {0, 10, 5, 1}, 5, DEFAULT_SERIAL_NUMBER, false, false, "24", false,
           {"/tmp/psdk_telemetry.sock", "/tmp/psdk_control.sock", NULL, NULL, "/tmp/polocalc_widget"}};

static volatile sig_atomic_t s_stop;
static volatile sig_atomic_t s_connected;   /* DjiCore_Init returned: a clean shutdown is possible */
static pthread_mutex_t       s_latestLock = PTHREAD_MUTEX_INITIALIZER;

static void OnSignal(int signalNum)
{
    static const char msg[] = "[" PROGRAM_NAME "] interrupted while connecting: exiting now\n";
    (void) signalNum;
    if (!s_connected) {
        /* DjiCore_Init blocks for up to ~40 s and can't be interrupted: leave at once. Exiting closes the
           port and releases its lock; only async-signal-safe calls here. */
        if (write(STDERR_FILENO, msg, sizeof(msg) - 1) < 0) { /* nothing to do */ }
        _exit(130);
    }
    s_stop = 1;
}

/* ---------------------------------------------------------------------------------------------- callbacks */
static T_DjiReturnCode OnTopicData(int index, const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts)
{
    T_Topic *t = &s_topics[index];

    TelemetryStream_Put(STREAM_RECORD_DATA, (uint16_t) t->topic, data, size,
                        ts ? ts->millisecond : 0, ts ? ts->microsecond : 0);

    pthread_mutex_lock(&s_latestLock);
    t->received++;
    if (size <= LATEST_MAX_SIZE) {
        memcpy(t->latest, data, size);
        t->haveLatest = true;
    }
    pthread_mutex_unlock(&s_latestLock);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* PSDK log messages go to stderr: stdout carries the binary stream */
static T_DjiReturnCode PrintConsole(const uint8_t *data, uint16_t dataLen)
{
    (void) dataLen;
    fputs((const char *) data, stderr);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* ---------------------------------------------------------------------------------------------- platform setup */
static T_DjiReturnCode PrepareSystemEnvironment(void)
{
    T_DjiOsalHandler osalHandler = {
        .TaskCreate = Osal_TaskCreate, .TaskDestroy = Osal_TaskDestroy, .TaskSleepMs = Osal_TaskSleepMs,
        .MutexCreate = Osal_MutexCreate, .MutexDestroy = Osal_MutexDestroy,
        .MutexLock = Osal_MutexLock, .MutexUnlock = Osal_MutexUnlock,
        .SemaphoreCreate = Osal_SemaphoreCreate, .SemaphoreDestroy = Osal_SemaphoreDestroy,
        .SemaphoreWait = Osal_SemaphoreWait, .SemaphoreTimedWait = Osal_SemaphoreTimedWait,
        .SemaphorePost = Osal_SemaphorePost,
        .Malloc = Osal_Malloc, .Free = Osal_Free,
        .GetRandomNum = Osal_GetRandomNum, .GetTimeMs = Osal_GetTimeMs, .GetTimeUs = Osal_GetTimeUs,
    };
    T_DjiLoggerConsole console = {
        .func = PrintConsole,
        .consoleLevel = s_opt.verbose ? DJI_LOGGER_CONSOLE_LOG_LEVEL_DEBUG : DJI_LOGGER_CONSOLE_LOG_LEVEL_INFO,
        .isSupportColor = false,
    };
    T_DjiHalUartHandler uartHandler = {
        .UartInit = HalUart_Init, .UartDeInit = HalUart_DeInit,
        .UartWriteData = HalUart_WriteData, .UartReadData = HalUart_ReadData,
        .UartGetStatus = HalUart_GetStatus, .UartGetDeviceInfo = HalUart_GetDeviceInfo,
    };
    T_DjiHalI2cHandler i2cHandler = {
        .I2cInit = HalI2c_Init, .I2cDeInit = HalI2c_DeInit,
        .I2cWriteData = HalI2c_WriteData, .I2cReadData = HalI2c_ReadData,
    };
    T_DjiFileSystemHandler fileSystemHandler = {
        .FileOpen = Osal_FileOpen, .FileClose = Osal_FileClose, .FileWrite = Osal_FileWrite,
        .FileRead = Osal_FileRead, .FileSync = Osal_FileSync, .FileSeek = Osal_FileSeek,
        .DirOpen = Osal_DirOpen, .DirClose = Osal_DirClose, .DirRead = Osal_DirRead,
        .Mkdir = Osal_Mkdir, .Unlink = Osal_Unlink, .Rename = Osal_Rename, .Stat = Osal_Stat,
    };

    if (DjiPlatform_RegOsalHandler(&osalHandler) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ||
        DjiLogger_AddConsole(&console) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ||
        DjiPlatform_RegHalI2cHandler(&i2cHandler) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ||
        DjiPlatform_RegHalUartHandler(&uartHandler) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ||
        DjiPlatform_RegFileSystemHandler(&fileSystemHandler) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static bool FillInUserInfo(T_DjiUserInfo *userInfo)
{
    memset(userInfo, 0, sizeof(*userInfo));
    if (strlen(USER_APP_NAME) >= sizeof(userInfo->appName) || strlen(USER_APP_ID) > sizeof(userInfo->appId) ||
        strlen(USER_APP_KEY) > sizeof(userInfo->appKey) || strlen(USER_APP_LICENSE) > sizeof(userInfo->appLicense) ||
        strlen(USER_DEVELOPER_ACCOUNT) >= sizeof(userInfo->developerAccount) ||
        strlen(USER_BAUD_RATE) > sizeof(userInfo->baudRate)) {
        TelemetryStream_Status(STREAM_STATE_ERROR, "application info in dji_sdk_app_info.h is too long");
        return false;
    }
    if (!strcmp(USER_APP_NAME, "your_app_name") || !strcmp(USER_APP_ID, "your_app_id") ||
        !strcmp(USER_APP_KEY, "your_app_key") || !strcmp(USER_APP_LICENSE, "your_app_license") ||
        !strcmp(USER_DEVELOPER_ACCOUNT, "your_developer_account")) {
        TelemetryStream_Status(STREAM_STATE_ERROR,
                               "application info not filled in: edit raspberry_pi/application/dji_sdk_app_info.h");
        return false;
    }
    strncpy(userInfo->appName, USER_APP_NAME, sizeof(userInfo->appName) - 1);
    memcpy(userInfo->appId, USER_APP_ID, strlen(USER_APP_ID));
    memcpy(userInfo->appKey, USER_APP_KEY, strlen(USER_APP_KEY));
    memcpy(userInfo->appLicense, USER_APP_LICENSE, strlen(USER_APP_LICENSE));
    memcpy(userInfo->baudRate, USER_BAUD_RATE, strlen(USER_BAUD_RATE));
    strncpy(userInfo->developerAccount, USER_DEVELOPER_ACCOUNT, sizeof(userInfo->developerAccount) - 1);
    return true;
}

/* The UART HAL locks the port so that only one PSDK program uses it. A program that was paused with Ctrl+Z,
   or is still running elsewhere, keeps that lock and the SDK then fails with an obscure error: check first. */
static bool CheckPortFree(const char *device)
{
    struct flock lock;
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd < 0) {
        TelemetryStream_Status(STREAM_STATE_ERROR, "cannot open %s: %s (USB gadget not active, or no permission)",
                               device, strerror(errno));
        return false;
    }
    memset(&lock, 0, sizeof(lock));
    lock.l_type = F_WRLCK;
    lock.l_whence = SEEK_SET;
    if (fcntl(fd, F_GETLK, &lock) == 0 && lock.l_type != F_UNLCK) {
        TelemetryStream_Status(STREAM_STATE_ERROR,
                               "%s is in use by process %d (another PSDK program, possibly paused with Ctrl+Z): "
                               "stop it first, e.g. 'kill %d'", device, (int) lock.l_pid, (int) lock.l_pid);
        close(fd);
        return false;
    }
    close(fd);
    return true;
}

/* ---------------------------------------------------------------------------------------------- high power */
/* --high-power value -> PSDK voltage; false for "off" or an invalid value (valid says which) */
static bool ToHighPowerVoltage(const char *value, E_DjiHighPowerVoltage *voltage, bool *valid)
{
    *valid = true;
    if (!strcmp(value, "24")) {
        *voltage = E_DJI_HIGH_POWER_VOLTAGE_24V;
        return true;
    }
    if (!strcmp(value, "17")) {
        *voltage = E_DJI_HIGH_POWER_VOLTAGE_17V;
        return true;
    }
    if (!strcmp(value, "13.6")) {
        *voltage = E_DJI_HIGH_POWER_VOLTAGE_13V6;
        return true;
    }
    *valid = !strcmp(value, "off");
    return false;
}

/* On the Matrice 400, HIGH_PWR_REQ (E-Port V2 pin 36) must be tied directly to GND in hardware (active low, no extra
   pull-down resistor): with it high or floating the aircraft accepts the request but keeps VCC at 13.6 V.
   Nothing to drive from the Pi. */
static T_DjiReturnCode WriteHighPowerApplyPin(E_DjiPowerManagementPinState pinState)
{
    (void) pinState;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Ask the aircraft for high power on the adapter's VCC pin, at the --high-power voltage. The aircraft doesn't
   remember it: it is requested at every connection. Only adapters that can choose the voltage are handled
   (E-Port V2 ribbon cable, SkyPort V3); a failure is reported but doesn't stop the telemetry. */
static void ApplyHighPower(const T_DjiAircraftInfoBaseInfo *aircraft, bool haveAircraft)
{
    E_DjiHighPowerVoltage voltage;
    T_DjiReturnCode rc;
    bool valid;

    if (!ToHighPowerVoltage(s_opt.highPower, &voltage, &valid)) {
        TelemetryStream_Status(STREAM_STATE_INFO, "high power off: VCC not changed");
        return;
    }
    if (!haveAircraft || (aircraft->djiAdapterType != DJI_SDK_ADAPTER_TYPE_EPORT_V2_RIBBON_CABLE &&
                          aircraft->djiAdapterType != DJI_SDK_ADAPTER_TYPE_SKYPORT_V3)) {
        TelemetryStream_Status(STREAM_STATE_WARNING, "high power: adapter type %d can't choose the VCC voltage, "
                               "VCC not changed", haveAircraft ? (int) aircraft->djiAdapterType : -1);
        return;
    }
    rc = DjiPowerManagement_Init();
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        rc = DjiPowerManagement_RegWriteHighPowerApplyPinCallback(WriteHighPowerApplyPin);
    }
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        rc = DjiPowerManagement_ApplyHighPowerSyncV2(voltage);     /* blocks ~600 ms */
    }
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        TelemetryStream_Status(STREAM_STATE_INFO, "high power requested: VCC %s V (the aircraft accepted the request)", s_opt.highPower);
    } else {
        TelemetryStream_Status(STREAM_STATE_ERROR, "cannot apply high power %s V (error 0x%08llX)",
                               s_opt.highPower, (unsigned long long) rc);
    }
}

/* ---------------------------------------------------------------------------------------------- subscription */
static bool ToFrequency(int hz, E_DjiDataSubscriptionTopicFreq *frequency)
{
    switch (hz) {
        case 1:   *frequency = DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ;   return true;
        case 5:   *frequency = DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ;   return true;
        case 10:  *frequency = DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ;  return true;
        case 50:  *frequency = DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ;  return true;
        case 100: *frequency = DJI_DATA_SUBSCRIPTION_TOPIC_100_HZ; return true;
        case 200: *frequency = DJI_DATA_SUBSCRIPTION_TOPIC_200_HZ; return true;
        case 400: *frequency = DJI_DATA_SUBSCRIPTION_TOPIC_400_HZ; return true;
        default:  return false;
    }
}

static void SendTopicTable(void)
{
    uint8_t payload[2 + TOPIC_COUNT * (7 + TELEMETRY_STREAM_NAME_SIZE)];
    uint8_t *p = payload + 2;

    payload[0] = (uint8_t) TOPIC_COUNT;
    payload[1] = 0;
    for (int i = 0; i < TOPIC_COUNT; i++) {
        uint16_t topic = (uint16_t) s_topics[i].topic, hz = (uint16_t) s_topics[i].frequency;
        p[0] = (uint8_t) topic; p[1] = (uint8_t) (topic >> 8);
        p[2] = (uint8_t) hz;    p[3] = (uint8_t) (hz >> 8);
        p[4] = (uint8_t) s_topics[i].size; p[5] = (uint8_t) (s_topics[i].size >> 8);
        p[6] = s_topics[i].subscribed ? 1 : 0;
        memset(p + 7, 0, TELEMETRY_STREAM_NAME_SIZE);
        strncpy((char *) p + 7, s_topics[i].name, TELEMETRY_STREAM_NAME_SIZE - 1);
        p += 7 + TELEMETRY_STREAM_NAME_SIZE;
    }
    TelemetryStream_Put(STREAM_RECORD_TOPIC_TABLE, 0, payload, (uint16_t) (p - payload), 0, 0);
}

/* Check the group rates and the PSDK limit of 242 bytes per rate. Groups with the same rate share it, so their
   sizes add up. Returns false and fills msg on error. */
static bool CheckGroups(char *msg, size_t msgSize)
{
    E_DjiDataSubscriptionTopicFreq unused;
    int bytes[GROUP_COUNT] = {0};

    for (int g = 0; g < GROUP_COUNT; g++) {
        if (s_opt.groupHz[g] != 0 && !ToFrequency(s_opt.groupHz[g], &unused)) {
            snprintf(msg, msgSize, "--%s %d: rates must be 0 (off) or one of 1, 5, 10, 50, 100, 200, 400 Hz",
                     s_groupNames[g], s_opt.groupHz[g]);
            return false;
        }
    }
    for (int i = 0; i < TOPIC_COUNT; i++) {
        bytes[s_topics[i].group] += s_topics[i].size;
    }
    for (int g = 0; g < GROUP_COUNT; g++) {
        int total = 0;
        if (s_opt.groupHz[g] == 0) {
            continue;
        }
        for (int h = 0; h < GROUP_COUNT; h++) {
            if (s_opt.groupHz[h] == s_opt.groupHz[g]) {
                total += bytes[h];
            }
        }
        if (total > SUBSCRIPTION_MAX_BYTES_PER_FREQ) {
            snprintf(msg, msgSize, "%d bytes of topics at %d Hz (max %d per rate): give the groups different rates",
                     total, s_opt.groupHz[g], SUBSCRIPTION_MAX_BYTES_PER_FREQ);
            return false;
        }
    }
    return true;
}

static int SubscribeTopics(void)
{
    int ok = 0;
    char msg[160];
    E_DjiDataSubscriptionTopicFreq frequency[GROUP_COUNT];

    if (!CheckGroups(msg, sizeof(msg))) {
        TelemetryStream_Status(STREAM_STATE_ERROR, "%s", msg);
        return -1;
    }
    for (int g = 0; g < GROUP_COUNT; g++) {
        if (s_opt.groupHz[g] != 0) {
            ToFrequency(s_opt.groupHz[g], &frequency[g]);
        }
    }

    if (DjiFcSubscription_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        TelemetryStream_Status(STREAM_STATE_ERROR, "DjiFcSubscription_Init failed");
        return -1;
    }
    for (int i = 0; i < TOPIC_COUNT; i++) {
        T_DjiReturnCode rc;
        if (s_opt.groupHz[s_topics[i].group] == 0) {
            continue;                            /* group turned off */
        }
        s_topics[i].frequency = frequency[s_topics[i].group];
        rc = DjiFcSubscription_SubscribeTopic(s_topics[i].topic, s_topics[i].frequency, s_topics[i].callback);
        s_topics[i].subscribed = (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
        if (s_topics[i].subscribed) {
            ok++;
        } else {
            /* not fatal: an aircraft may not support every topic */
            TelemetryStream_Status(STREAM_STATE_WARNING, "cannot subscribe %s at %d Hz (error 0x%08llX)",
                                   s_topics[i].name, (int) s_topics[i].frequency, (unsigned long long) rc);
        }
    }
    SendTopicTable();
    return ok;
}

static void UnsubscribeTopics(void)
{
    /* the PSDK requires unsubscribing in the order of subscription */
    for (int i = 0; i < TOPIC_COUNT; i++) {
        if (s_topics[i].subscribed) {
            DjiFcSubscription_UnSubscribeTopic(s_topics[i].topic);
            s_topics[i].subscribed = false;
        }
    }
    DjiFcSubscription_DeInit();
}

/* ---------------------------------------------------------------------------------------------- reporting */
static void SendStats(void)
{
    uint8_t payload[4 + 2 + TOPIC_COUNT * 6];
    uint8_t *p = payload + 6;
    uint32_t dropped = TelemetryStream_Dropped();

    for (int i = 0; i < 4; i++) {
        payload[i] = (uint8_t) (dropped >> (8 * i));
    }
    payload[4] = (uint8_t) TOPIC_COUNT;
    payload[5] = 0;

    fprintf(stderr, "[%s] received:", PROGRAM_NAME);
    pthread_mutex_lock(&s_latestLock);
    for (int i = 0; i < TOPIC_COUNT; i++) {
        uint16_t topic = (uint16_t) s_topics[i].topic;
        uint32_t n = s_topics[i].received;
        p[0] = (uint8_t) topic; p[1] = (uint8_t) (topic >> 8);
        for (int b = 0; b < 4; b++) {
            p[2 + b] = (uint8_t) (n >> (8 * b));
        }
        p += 6;
        if (s_topics[i].subscribed) {
            fprintf(stderr, " %s=%u", s_topics[i].name, n);
        }
    }
    pthread_mutex_unlock(&s_latestLock);
    fprintf(stderr, " dropped=%u\n", dropped);
    TelemetryStream_Put(STREAM_RECORD_STATS, 0, payload, (uint16_t) sizeof(payload), 0, 0);
}

/* copy the latest sample of a topic; false if none was received yet */
static bool GetLatest(E_DjiFcSubscriptionTopic topic, void *dst, size_t size)
{
    bool have = false;

    memset(dst, 0, size);
    pthread_mutex_lock(&s_latestLock);
    for (int i = 0; i < TOPIC_COUNT; i++) {
        if (s_topics[i].topic == topic && s_topics[i].haveLatest) {
            memcpy(dst, s_topics[i].latest, size);
            have = true;
            break;
        }
    }
    pthread_mutex_unlock(&s_latestLock);
    return have;
}

static void PrintBattery(const char *label, const T_DjiFcSubscriptionSingleBatteryInfo *b)
{
    fprintf(stderr, "  %-9s %3u %%  %6.2f V  %7.2f A  %5u/%5u mAh  %5.1f C  cells %u  closed_reason %u  "
                    "self_check %u  cell_break %u  comm_abnormal %u\n",
            label, b->batteryCapacityPercent, b->currentVoltage / 1000.0, b->currentElectric / 1000.0,
            b->remainedCapacity, b->fullCapacity, b->batteryTemperature / 10.0, b->cellCount,
            b->batteryState.batteryClosedReason, b->batteryState.selfCheckError, b->batteryState.cellBreak,
            b->batteryState.batteryCommunicationAbnormal);
}

/* --print: decode the latest values, for testing on the bench */
static void PrintLatest(void)
{
    T_DjiFcSubscriptionQuaternion q;
    T_DjiFcSubscriptionRtkPosition rtk;
    T_DjiFcSubscriptionGpsPosition gps;
    T_DjiFcSubscriptionSingleBatteryInfo battery1, battery2;
    T_DjiFcSubscriptionEscData esc;
    T_DjiFcSubscriptionFlightAnomaly anomaly;
    T_DjiFcSubscriptionRtkYaw rtkYaw;
    T_DjiFcSubscriptionRtkPositionInfo rtkInfo;
    T_DjiFcSubscriptionRtkYawInfo rtkYawInfo;
    T_DjiFcSubscriptionFlightStatus flightStatus;
    T_DjiFcSubscriptionDisplaymode displayMode;
    uint32_t anomalyBits;
    bool haveStatus, haveMode;

    if (GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, &q, sizeof(q))) {
        double roll = atan2(2.0 * (q.q0 * q.q1 + q.q2 * q.q3), 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2));
        double sinp = 2.0 * (q.q0 * q.q2 - q.q3 * q.q1);
        double pitch = fabs(sinp) >= 1.0 ? copysign(M_PI / 2, sinp) : asin(sinp);
        double yaw = atan2(2.0 * (q.q0 * q.q3 + q.q1 * q.q2), 1.0 - 2.0 * (q.q2 * q.q2 + q.q3 * q.q3));
        fprintf(stderr, "  attitude  roll %7.2f  pitch %7.2f  yaw %7.2f deg\n",
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    }
    {
        T_DjiFcSubscriptionAccelerationRaw acc;
        T_DjiFcSubscriptionAngularRateRaw gyro;
        T_DjiFcSubscriptionCompass mag;
        bool haveAcc = GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW, &acc, sizeof(acc));
        bool haveGyro = GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW, &gyro, sizeof(gyro));
        bool haveMag = GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS, &mag, sizeof(mag));
        if (haveAcc || haveGyro || haveMag) {
            fprintf(stderr, "  imu       acc %+7.3f %+7.3f %+7.3f m/s2  gyro %+7.4f %+7.4f %+7.4f rad/s  mag %6d %6d %6d\n",
                    acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, mag.x, mag.y, mag.z);
        }
    }
    {
        T_DjiFcSubscriptionPositionFused pos;
        T_DjiFcSubscriptionGpsDate date;
        T_DjiFcSubscriptionGpsTime gpsTime;
        if (GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, &pos, sizeof(pos))) {
            GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, &date, sizeof(date));
            GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, &gpsTime, sizeof(gpsTime));
            fprintf(stderr, "  fused     lat %.8f  lon %.8f  alt %.2f m  sats %u   gps time %08u %06u\n",
                    pos.latitude * 180.0 / M_PI, pos.longitude * 180.0 / M_PI, pos.altitude,
                    pos.visibleSatelliteNumber, date, gpsTime);
        }
    }
    if (GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, &rtk, sizeof(rtk))) {
        GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, &rtkInfo, sizeof(rtkInfo));
        GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW, &rtkYaw, sizeof(rtkYaw));
        GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO, &rtkYawInfo, sizeof(rtkYawInfo));
        fprintf(stderr, "  rtk       lat %.8f  lon %.8f  hmsl %.3f m  pos_info %u  yaw %d  yaw_info %u\n",
                rtk.latitude, rtk.longitude, rtk.hfsl, rtkInfo, rtkYaw, rtkYawInfo);
    }
    if (GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, &gps, sizeof(gps))) {
        fprintf(stderr, "  gps (raw) x %d  y %d  z %d\n", gps.x, gps.y, gps.z);
    }
    haveStatus = GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, &flightStatus, sizeof(flightStatus));
    haveMode = GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, &displayMode, sizeof(displayMode));
    if (GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_FLIGHT_ANOMALY, &anomaly, sizeof(anomaly))) {
        memcpy(&anomalyBits, &anomaly, sizeof(anomalyBits));
    } else {
        anomalyBits = 0;
    }
    if (haveStatus || haveMode) {
        fprintf(stderr, "  status    flight_status %u  display_mode %u  anomaly 0x%05X%s%s%s%s%s\n",
                flightStatus, displayMode, (unsigned) anomalyBits,
                anomaly.escTemperatureHigh ? " ESC_TEMP_HIGH" : "",
                anomaly.atLeastOneEscDisconnected ? " ESC_DISCONNECTED" : "",
                anomaly.aircraftIsFalling ? " FALLING" : "",
                anomaly.strongWindLevel2 ? " WIND_LEVEL2" : (anomaly.strongWindLevel1 ? " WIND_LEVEL1" : ""),
                anomaly.gpsYawError ? " GPS_YAW_ERROR" : "");
    }
    if (GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1, &battery1, sizeof(battery1))) {
        PrintBattery("battery1", &battery1);
    }
    if (GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2, &battery2, sizeof(battery2))) {
        PrintBattery("battery2", &battery2);
    }
    if (GetLatest(DJI_FC_SUBSCRIPTION_TOPIC_ESC_DATA, &esc, sizeof(esc))) {
        int shown = 0;
        for (int i = 0; i < 8; i++) {
            const ESCStatusIndividual *e = &esc.esc[i];
            if (e->voltage == 0 && e->speed == 0 && e->current == 0 && e->temperature == 0) {
                continue;                        /* slot not used by this aircraft */
            }
            fprintf(stderr, "  esc%d      %6d rpm  %6.2f A  %6.2f V  %4d C%s%s%s%s%s\n", i + 1, e->speed,
                    e->current / 1000.0, e->voltage / 1000.0, e->temperature,
                    e->stall ? " STALL" : "", e->empty ? " NO_LOAD" : "", e->unbalanced ? " UNBALANCED" : "",
                    e->escDisconnected ? " DISCONNECTED" : "", e->temperatureHigh ? " TEMP_HIGH" : "");
            shown++;
        }
        if (!shown) {
            fprintf(stderr, "  esc       all zero\n");
        }
    }
}

/* ---------------------------------------------------------------------------------------------- main */
static void Usage(void)
{
    fprintf(stderr,
            "Usage: %s [options]\n"
            "  -o, --output FILE   write the binary stream to FILE ('-' = stdout, default)\n"
            "      --imu HZ        rate of the imu group: IMU navigation data, 50 Hz only (default 0 = off)\n"
            "      --fast HZ       rate of the fast group: attitude, IMU, GPS, RTK, positions, RC (default 10)\n"
            "      --status HZ     rate of the status group: flight state, ESCs, GPS quality (default 5)\n"
            "      --slow HZ       rate of the slow group: batteries, GPS date/time, home point (default 1)\n"
            "      --stats S       seconds between STATS records (default 5, 0 = off)\n"
            "      --serial SN     payload serial number reported to the aircraft (default %s)\n"
            "  -p, --print         print the latest decoded values to stderr every second\n"
            "      --high-power V  E-Port VCC voltage to request: 24, 17, 13.6 or off (default 24)\n"
            "      --service       run as psdk-bridge.service: telemetry socket, control socket, RC widgets\n"
            "      --socket PATH   service: telemetry socket (default /tmp/psdk_telemetry.sock)\n"
            "      --control PATH  service: control socket for telemd (default /tmp/psdk_control.sock)\n"
            "      --config-dir D  service: porter's config/ folder, listed on the RC\n"
            "      --widget-icons D  service: folder with the widget icons (raspberry_pi_telemetry/widget)\n"
            "  -v, --verbose       PSDK debug logs\n"
            "Rates: 0 (off), 1, 5, 10, 50, 100, 200 or 400 Hz; max 242 bytes of topics per rate.\n", PROGRAM_NAME, DEFAULT_SERIAL_NUMBER);
}

static bool ParseArgs(int argc, char **argv)
{
    static const struct option options[] = {
        {"output", required_argument, NULL, 'o'}, {"imu", required_argument, NULL, 'i'},
        {"fast", required_argument, NULL, 'f'},   {"status", required_argument, NULL, 'u'},
        {"slow", required_argument, NULL, 's'},   {"stats", required_argument, NULL, 't'},
        {"serial", required_argument, NULL, 'n'}, {"print", no_argument, NULL, 'p'},
        {"verbose", no_argument, NULL, 'v'},      {"help", no_argument, NULL, 'h'},
        {"high-power", required_argument, NULL, 'w'},
        {"service", no_argument, NULL, 'S'},      {"socket", required_argument, NULL, 'K'},
        {"control", required_argument, NULL, 'C'}, {"config-dir", required_argument, NULL, 'D'},
        {"widget-icons", required_argument, NULL, 'I'},
        {NULL, 0, NULL, 0},
    };
    int c;
    while ((c = getopt_long(argc, argv, "o:pvh", options, NULL)) != -1) {
        switch (c) {
            case 'o': s_opt.output = optarg; break;
            case 'i': s_opt.groupHz[GROUP_IMU] = atoi(optarg); break;
            case 'f': s_opt.groupHz[GROUP_FAST] = atoi(optarg); break;
            case 'u': s_opt.groupHz[GROUP_STATUS] = atoi(optarg); break;
            case 's': s_opt.groupHz[GROUP_SLOW] = atoi(optarg); break;
            case 't': s_opt.statsS = atoi(optarg); break;
            case 'n': s_opt.serial = optarg; break;
            case 'p': s_opt.print = true; break;
            case 'v': s_opt.verbose = true; break;
            case 'w': s_opt.highPower = optarg; break;
            case 'S': s_opt.service = true; break;
            case 'K': s_opt.serviceOptions.telemetrySocket = optarg; break;
            case 'C': s_opt.serviceOptions.controlSocket = optarg; break;
            case 'D': s_opt.serviceOptions.configDir = optarg; break;
            case 'I': s_opt.serviceOptions.widgetTemplateDir = optarg; break;
            default:  Usage(); return false;
        }
    }
    /* check the rates now, not after the (slow) connection to the aircraft */
    {
        char msg[160];
        if (!CheckGroups(msg, sizeof(msg))) {
            fprintf(stderr, "%s: %s\n", PROGRAM_NAME, msg);
            return false;
        }
    }
    {
        E_DjiHighPowerVoltage unused;
        bool valid;
        ToHighPowerVoltage(s_opt.highPower, &unused, &valid);
        if (!valid) {
            fprintf(stderr, "%s: --high-power must be 24, 17, 13.6 or off\n", PROGRAM_NAME);
            return false;
        }
    }
    if (s_opt.service && s_opt.serviceOptions.widgetTemplateDir == NULL) {
        fprintf(stderr, "%s: --service needs --widget-icons (the raspberry_pi_telemetry/widget folder)\n", PROGRAM_NAME);
        return false;
    }
    if (s_opt.statsS < 0) {
        s_opt.statsS = 0;
    }
    return true;
}

int main(int argc, char **argv)
{
    T_DjiUserInfo userInfo;
    T_DjiAircraftInfoBaseInfo aircraft;
    bool haveAircraft;
    T_DjiFirmwareVersion firmwareVersion = {.majorVersion = 1, .minorVersion = 0};
    struct sigaction sa;
    int outputFd, exitCode = 0;
    unsigned ticks = 0, lastDataTick = 0;
    uint32_t lastFastReceived = 0;

    if (!ParseArgs(argc, argv)) {
        return 1;
    }

    /* the binary stream goes to its own descriptor; stdout itself is pointed to stderr, so that any
       printf() inside the SDK or the HAL can never corrupt the stream */
    if (s_opt.service) {
        outputFd = -1;                           /* the records go to the telemetry socket (Service_StreamSink) */
    } else if (strcmp(s_opt.output, "-") == 0 && isatty(STDOUT_FILENO)) {
        /* binary records on a terminal are just garbage: discard them */
        fprintf(stderr, "[%s] stdout is a terminal: binary stream discarded "
                        "(use --output FILE, or pipe it into tools/psdk_stream_reader.py)\n", PROGRAM_NAME);
        outputFd = open("/dev/null", O_WRONLY);
    } else if (strcmp(s_opt.output, "-") == 0) {
        outputFd = dup(STDOUT_FILENO);
    } else {
        outputFd = open(s_opt.output, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    }
    if (outputFd < 0 && !s_opt.service) {
        fprintf(stderr, "[%s] cannot open output %s: %s\n", PROGRAM_NAME, s_opt.output, strerror(errno));
        return 1;
    }
    dup2(STDERR_FILENO, STDOUT_FILENO);
    setvbuf(stderr, NULL, _IOLBF, 0);

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = OnSignal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);                    /* a closed reader shows up as a write error instead */

    if (s_opt.service ? !TelemetryStream_StartWithSink(Service_StreamSink) : !TelemetryStream_Start(outputFd)) {
        fprintf(stderr, "[%s] cannot start the stream writer\n", PROGRAM_NAME);
        return 1;
    }
    TelemetryStream_Status(STREAM_STATE_STARTING, "%s starting (stream format v%d)", PROGRAM_NAME,
                           TELEMETRY_STREAM_VERSION);

    if (PrepareSystemEnvironment() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        TelemetryStream_Status(STREAM_STATE_ERROR, "platform setup failed");
        exitCode = 2;
        goto stop_stream;
    }
    if (!FillInUserInfo(&userInfo)) {
        exitCode = 2;
        goto stop_stream;
    }
    DjiCore_SetFirmwareVersion(firmwareVersion);
    DjiCore_SetSerialNumber(s_opt.serial);

    if (!CheckPortFree(LINUX_UART_DEV1)) {
        exitCode = 3;
        goto stop_stream;
    }

    TelemetryStream_Status(STREAM_STATE_CONNECTING, "connecting to the aircraft (UART %s, %s baud)",
                           LINUX_UART_DEV1, USER_BAUD_RATE);
    if (DjiCore_Init(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        TelemetryStream_Status(STREAM_STATE_ERROR, "DjiCore_Init failed: no aircraft, wrong link or authentication error");
        exitCode = 3;
        goto stop_stream;
    }
    s_connected = 1;                             /* from here on, SIGINT/SIGTERM stop cleanly */
    DjiCore_SetAlias("POLOCALC");
    haveAircraft = DjiAircraftInfo_GetBaseInfo(&aircraft) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    if (haveAircraft) {
        TelemetryStream_Status(STREAM_STATE_CONNECTED, "connected: aircraft type %d%s, adapter type %d, mount position %d",
                               (int) aircraft.aircraftType,
                               aircraft.aircraftType == DJI_AIRCRAFT_TYPE_M400 ? " (Matrice 400)" : "",
                               (int) aircraft.djiAdapterType, (int) aircraft.mountPosition);
    } else {
        TelemetryStream_Status(STREAM_STATE_CONNECTED, "connected (aircraft info not available)");
    }
    ApplyHighPower(&aircraft, haveAircraft);
    if (s_opt.service && !Service_StartWidgets(&s_opt.serviceOptions)) {
        TelemetryStream_Status(STREAM_STATE_WARNING, "RC widgets not available (see the log above)");
    }
    DjiCore_ApplicationStart();

    if (SubscribeTopics() <= 0) {
        TelemetryStream_Status(STREAM_STATE_ERROR, "no topic could be subscribed");
        exitCode = 4;
        goto deinit_core;
    }
    TelemetryStream_Status(STREAM_STATE_SUBSCRIBED, "telemetry running: imu %d Hz, fast %d Hz, status %d Hz, slow %d Hz",
                           s_opt.groupHz[GROUP_IMU], s_opt.groupHz[GROUP_FAST], s_opt.groupHz[GROUP_STATUS],
                           s_opt.groupHz[GROUP_SLOW]);
    if (s_opt.service && !Service_StartSockets(&s_opt.serviceOptions)) {
        TelemetryStream_Status(STREAM_STATE_ERROR, "cannot open the service sockets");
        exitCode = 2;
        s_stop = 1;
    }

    while (!s_stop && !TelemetryStream_Failed()) {
        usleep(100 * 1000);
        ticks++;
        /* the PSDK doesn't recover a lost link by itself: exit, so that lager starts the program again */
        if (ticks % 10 == 0) {
            uint32_t fastReceived = 0;
            pthread_mutex_lock(&s_latestLock);
            for (int i = 0; i < TOPIC_COUNT; i++) {
                if (s_topics[i].group == GROUP_FAST) {
                    fastReceived += s_topics[i].received;
                }
            }
            pthread_mutex_unlock(&s_latestLock);
            if (fastReceived != lastFastReceived) {
                lastFastReceived = fastReceived;
                lastDataTick = ticks;
            } else if (ticks - lastDataTick >= LINK_TIMEOUT_S * 10) {
                TelemetryStream_Status(STREAM_STATE_ERROR, "no telemetry for %d s: link to the aircraft lost",
                                       LINK_TIMEOUT_S);
                exitCode = 6;
                break;
            }
        }
        if (s_opt.statsS > 0 && ticks % (unsigned) (s_opt.statsS * 10) == 0) {
            SendStats();
        }
        if (s_opt.print && ticks % 10 == 0) {
            PrintLatest();
        }
    }
    if (exitCode == 0 && TelemetryStream_Failed()) {
        exitCode = 5;                            /* the reader went away */
    }

    TelemetryStream_Status(STREAM_STATE_STOPPING, "stopping");
    SendStats();                                 /* final counts, while the topics are still marked subscribed */
    UnsubscribeTopics();
deinit_core:
    DjiCore_DeInit();
stop_stream:
    TelemetryStream_Stop();                      /* the last records (STOPPING) reach the readers first */
    if (s_opt.service) {
        Service_Stop();
    } else {
        close(outputFd);
    }
    return exitCode;
}
