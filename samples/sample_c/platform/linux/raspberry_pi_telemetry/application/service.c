/**
 * service.c - psdk_telemetry service mode: telemetry and control sockets, DJI Pilot 2 widgets. See service.h.
 * All the widgets are on the main panel: Pilot 2 didn't show a list placed in the payload settings (config_interface).
 */
#define _GNU_SOURCE                 /* pthread_setname_np() */
#include "service.h"
#include "telemetry_stream.h"

#include <dji_typedef.h>
#include <dji_widget.h>

#include <dirent.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define RECORD_SIZE_MAX      (TELEMETRY_STREAM_HEADER_SIZE + TELEMETRY_STREAM_MAX_PAYLOAD + TELEMETRY_STREAM_CRC_SIZE)
#define MAX_TELEMETRY_CLIENTS 8
#define CLIENT_SEND_BUFFER   (1024 * 1024)
#define CLIENT_SEND_TIMEOUT_MS 200        /* a reader this slow is disconnected */
#define MAX_CONFIGS          16
#define CONFIG_NAME_SIZE     48
#define CONTROL_LINE_SIZE    512
#define MESSAGE_HOLD_S       4            /* a MSG stays on the RC this long before the status line comes back */
#define SHUTDOWN_CONFIRM_S   5            /* Shutdown must be pressed a second time within this many seconds */
#define PATH_SIZE            256

enum { WIDGET_START = 0, WIDGET_STOP = 1, WIDGET_POI = 2, WIDGET_CONFIG = 3, WIDGET_SHUTDOWN = 4 };

static volatile bool s_stop;

/* ---------------------------------------------------------------------------------------------- telemetry socket */
static pthread_mutex_t s_clientsLock = PTHREAD_MUTEX_INITIALIZER;
static int             s_clients[MAX_TELEMETRY_CLIENTS];
static int             s_clientCount;
static int             s_telemetryListen = -1;
static pthread_t       s_telemetryThread;
static bool            s_telemetryRunning;
static char            s_telemetryPath[PATH_SIZE];

/* records a new reader needs before the live stream */
static uint8_t s_connectedRecord[RECORD_SIZE_MAX], s_tableRecord[RECORD_SIZE_MAX], s_subscribedRecord[RECORD_SIZE_MAX];
static size_t  s_connectedLength, s_tableLength, s_subscribedLength;

static bool SendAll(int fd, const uint8_t *data, size_t length)
{
    while (length > 0) {
        ssize_t n = send(fd, data, length, MSG_NOSIGNAL);
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            return false;                       /* gone, or too slow (SO_SNDTIMEO) */
        }
        data += n;
        length -= (size_t) n;
    }
    return true;
}

static void Cache(uint8_t *dst, size_t *dstLength, const uint8_t *record, size_t length)
{
    if (length <= RECORD_SIZE_MAX) {
        memcpy(dst, record, length);
        *dstLength = length;
    }
}

void Service_StreamSink(const uint8_t *record, size_t length)
{
    uint8_t type = record[3];

    pthread_mutex_lock(&s_clientsLock);
    if (type == STREAM_RECORD_TOPIC_TABLE) {
        Cache(s_tableRecord, &s_tableLength, record, length);
    } else if (type == STREAM_RECORD_STATUS && length > TELEMETRY_STREAM_HEADER_SIZE) {
        uint8_t state = record[TELEMETRY_STREAM_HEADER_SIZE];
        if (state == STREAM_STATE_CONNECTED) {
            Cache(s_connectedRecord, &s_connectedLength, record, length);
        } else if (state == STREAM_STATE_SUBSCRIBED) {
            Cache(s_subscribedRecord, &s_subscribedLength, record, length);
        }
    }
    for (int i = 0; i < s_clientCount;) {
        if (SendAll(s_clients[i], record, length)) {
            i++;
            continue;
        }
        fprintf(stderr, "[psdk_telemetry] telemetry reader disconnected\n");
        close(s_clients[i]);
        s_clients[i] = s_clients[--s_clientCount];
    }
    pthread_mutex_unlock(&s_clientsLock);
}

static int ListenOn(const char *path)
{
    struct sockaddr_un address;
    int fd;

    if (strlen(path) >= sizeof(address.sun_path)) {
        fprintf(stderr, "[psdk_telemetry] socket path too long: %s\n", path);
        return -1;
    }
    fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        return -1;
    }
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, path);
    unlink(path);
    if (bind(fd, (struct sockaddr *) &address, sizeof(address)) < 0 || listen(fd, 4) < 0) {
        fprintf(stderr, "[psdk_telemetry] cannot listen on %s: %s\n", path, strerror(errno));
        close(fd);
        return -1;
    }
    chmod(path, 0660);
    return fd;
}

static void *TelemetryAcceptThread(void *argument)
{
    (void) argument;
    while (!s_stop) {
        struct pollfd p = {.fd = s_telemetryListen, .events = POLLIN};
        struct timeval timeout = {.tv_sec = 0, .tv_usec = CLIENT_SEND_TIMEOUT_MS * 1000};
        int buffer = CLIENT_SEND_BUFFER;
        int fd;

        if (poll(&p, 1, 500) <= 0) {
            continue;
        }
        fd = accept4(s_telemetryListen, NULL, NULL, SOCK_CLOEXEC);
        if (fd < 0) {
            continue;
        }
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &buffer, sizeof(buffer));

        pthread_mutex_lock(&s_clientsLock);
        if (s_clientCount >= MAX_TELEMETRY_CLIENTS ||
            (s_connectedLength && !SendAll(fd, s_connectedRecord, s_connectedLength)) ||
            (s_tableLength && !SendAll(fd, s_tableRecord, s_tableLength)) ||
            (s_subscribedLength && !SendAll(fd, s_subscribedRecord, s_subscribedLength))) {
            close(fd);
        } else {
            s_clients[s_clientCount++] = fd;
            fprintf(stderr, "[psdk_telemetry] telemetry reader connected (%d)\n", s_clientCount);
        }
        pthread_mutex_unlock(&s_clientsLock);
    }
    return NULL;
}

/* ---------------------------------------------------------------------------------------------- control socket */
static pthread_mutex_t s_controlLock = PTHREAD_MUTEX_INITIALIZER;
static int             s_controlListen = -1;
static int             s_control = -1;           /* telemd */
static pthread_t       s_controlThread;
static bool            s_controlRunning;
static char            s_controlPath[PATH_SIZE];

/* state shown by the widgets */
static volatile int32_t s_poiOn;                 /* from telemd's STATE line */
static volatile int32_t s_configIndex;
static char             s_configs[MAX_CONFIGS][CONFIG_NAME_SIZE];
static int              s_configCount;
static volatile time_t  s_messageUntil;          /* a MSG is shown until then */
static time_t           s_shutdownArmedUntil;    /* first Shutdown press: a second one before then confirms */
static bool             s_widgetsReady;

static void ShowOnRc(const char *text)
{
    char message[DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN];
    if (!s_widgetsReady) {
        return;
    }
    size_t n = strlen(text);
    if (n >= sizeof(message)) {
        n = sizeof(message) - 1;                /* the RC shows at most 255 characters */
    }
    memcpy(message, text, n);
    message[n] = '\0';
    DjiWidgetFloatingWindow_ShowMessage(message);
}

/* Send a command line to telemd; never blocks (called from the widget callbacks). */
static void SendToTelemd(const char *format, ...)
{
    char line[CONTROL_LINE_SIZE];
    va_list args;
    bool sent = false;

    va_start(args, format);
    vsnprintf(line, sizeof(line) - 1, format, args);
    va_end(args);
    strcat(line, "\n");

    pthread_mutex_lock(&s_controlLock);
    if (s_control >= 0) {
        sent = send(s_control, line, strlen(line), MSG_NOSIGNAL | MSG_DONTWAIT) == (ssize_t) strlen(line);
    }
    pthread_mutex_unlock(&s_controlLock);

    line[strlen(line) - 1] = '\0';
    fprintf(stderr, "[psdk_telemetry] RC command: %s%s\n", line, sent ? "" : " (telemd not connected)");
    if (!sent) {
        ShowOnRc("telemd not connected: command not sent");
        s_messageUntil = time(NULL) + MESSAGE_HOLD_S;
    }
}

static void HandleControlLine(char *line)
{
    if (!strncmp(line, "TEXT ", 5)) {
        if (time(NULL) >= s_messageUntil) {
            ShowOnRc(line + 5);
        }
    } else if (!strncmp(line, "MSG ", 4)) {
        ShowOnRc(line + 4);
        s_messageUntil = time(NULL) + MESSAGE_HOLD_S;
    } else if (!strncmp(line, "STATE ", 6)) {
        int running = 0, poi = 0;
        if (sscanf(line + 6, "%d %d", &running, &poi) == 2) {
            s_poiOn = poi ? DJI_WIDGET_SWITCH_STATE_ON : DJI_WIDGET_SWITCH_STATE_OFF;
        }
    }
}

static void *ControlThread(void *argument)
{
    char buffer[CONTROL_LINE_SIZE];
    size_t used = 0;
    (void) argument;

    while (!s_stop) {
        struct pollfd p[2] = {{.fd = s_controlListen, .events = POLLIN}, {.fd = s_control, .events = POLLIN}};
        int count = s_control >= 0 ? 2 : 1;

        if (poll(p, (nfds_t) count, 500) <= 0) {
            continue;
        }
        if (p[0].revents & POLLIN) {
            int fd = accept4(s_controlListen, NULL, NULL, SOCK_CLOEXEC);
            if (fd >= 0) {
                pthread_mutex_lock(&s_controlLock);
                if (s_control >= 0) {
                    close(s_control);           /* a new telemd replaces the old connection */
                }
                s_control = fd;
                used = 0;
                pthread_mutex_unlock(&s_controlLock);
                fprintf(stderr, "[psdk_telemetry] telemd connected to the control socket\n");
            }
            continue;
        }
        if (count == 2 && (p[1].revents & (POLLIN | POLLHUP | POLLERR))) {
            ssize_t n = recv(s_control, buffer + used, sizeof(buffer) - 1 - used, 0);
            if (n <= 0) {
                pthread_mutex_lock(&s_controlLock);
                close(s_control);
                s_control = -1;
                pthread_mutex_unlock(&s_controlLock);
                fprintf(stderr, "[psdk_telemetry] telemd disconnected from the control socket\n");
                continue;
            }
            used += (size_t) n;
            buffer[used] = '\0';
            for (char *end; (end = strchr(buffer, '\n')) != NULL;) {
                *end = '\0';
                HandleControlLine(buffer);
                used -= (size_t) (end + 1 - buffer);
                memmove(buffer, end + 1, used + 1);
            }
            if (used >= sizeof(buffer) - 1) {
                used = 0;                       /* line too long: drop it */
            }
        }
    }
    return NULL;
}

/* ---------------------------------------------------------------------------------------------- widgets */
static T_DjiReturnCode SetWidgetValue(E_DjiWidgetType type, uint32_t index, int32_t value, void *userData)
{
    (void) type;
    (void) userData;
    switch (index) {
        case WIDGET_START:
            if (value == DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
                SendToTelemd("CMD start %s", s_configCount ? s_configs[s_configIndex] : "default");
            }
            break;
        case WIDGET_STOP:
            if (value == DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
                SendToTelemd("CMD stop");
            }
            break;
        case WIDGET_POI:
            s_poiOn = value;                    /* follow the pilot at once; telemd's STATE line then confirms */
            SendToTelemd(value == DJI_WIDGET_SWITCH_STATE_ON ? "CMD poi on" : "CMD poi off");
            break;
        case WIDGET_SHUTDOWN:
            if (value == DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
                time_t now = time(NULL);
                if (now <= s_shutdownArmedUntil) {
                    s_shutdownArmedUntil = 0;
                    SendToTelemd("CMD shutdown");
                } else {
                    char message[96];
                    s_shutdownArmedUntil = now + SHUTDOWN_CONFIRM_S;
                    snprintf(message, sizeof(message), "Press Shutdown again within %d s to shut down the Pi",
                             SHUTDOWN_CONFIRM_S);
                    ShowOnRc(message);
                    s_messageUntil = now + SHUTDOWN_CONFIRM_S;
                    fprintf(stderr, "[psdk_telemetry] RC: shutdown pressed, waiting for confirmation\n");
                }
            }
            break;
        case WIDGET_CONFIG:
            if (value >= 0 && value < s_configCount) {
                s_configIndex = value;
                fprintf(stderr, "[psdk_telemetry] RC configuration: %s\n", s_configs[value]);
            }
            break;
        default:
            break;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode GetWidgetValue(E_DjiWidgetType type, uint32_t index, int32_t *value, void *userData)
{
    (void) type;
    (void) userData;
    switch (index) {
        case WIDGET_POI:    *value = s_poiOn; break;
        case WIDGET_CONFIG: *value = s_configIndex; break;
        default:            *value = 0; break;         /* buttons: released */
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static const T_DjiWidgetHandlerListItem s_widgetHandlers[] = {
    {WIDGET_START,  DJI_WIDGET_TYPE_BUTTON, SetWidgetValue, GetWidgetValue, NULL},
    {WIDGET_STOP,   DJI_WIDGET_TYPE_BUTTON, SetWidgetValue, GetWidgetValue, NULL},
    {WIDGET_POI,    DJI_WIDGET_TYPE_SWITCH, SetWidgetValue, GetWidgetValue, NULL},
    {WIDGET_CONFIG, DJI_WIDGET_TYPE_LIST,   SetWidgetValue, GetWidgetValue, NULL},
    {WIDGET_SHUTDOWN, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue, GetWidgetValue, NULL},
};

static int CompareNames(const void *a, const void *b)
{
    return strcmp((const char *) a, (const char *) b);
}

/* porter configurations: config/<name>.yml, names porter's telemd accepts */
static void ListConfigs(const char *configDir)
{
    DIR *dir = configDir ? opendir(configDir) : NULL;
    struct dirent *entry;

    s_configCount = 0;
    if (dir != NULL) {
        while ((entry = readdir(dir)) != NULL && s_configCount < MAX_CONFIGS) {
            size_t n = strlen(entry->d_name);
            bool valid = n > 4 && n - 4 < CONFIG_NAME_SIZE && !strcmp(entry->d_name + n - 4, ".yml");
            for (size_t i = 0; valid && i < n - 4; i++) {
                char c = entry->d_name[i];
                valid = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
            }
            if (valid) {
                memcpy(s_configs[s_configCount], entry->d_name, n - 4);
                s_configs[s_configCount][n - 4] = '\0';
                s_configCount++;
            }
        }
        closedir(dir);
    }
    if (s_configCount == 0) {
        strcpy(s_configs[0], "default");
        s_configCount = 1;
    }
    qsort(s_configs, (size_t) s_configCount, CONFIG_NAME_SIZE, CompareNames);
    s_configIndex = 0;
    for (int i = 0; i < s_configCount; i++) {
        if (!strcmp(s_configs[i], "default")) {
            s_configIndex = i;
        }
    }
}

static bool CopyFile(const char *from, const char *to)
{
    char data[4096];
    size_t n;
    FILE *in = fopen(from, "rb"), *out = in ? fopen(to, "wb") : NULL;
    bool ok = in && out;

    while (ok && (n = fread(data, 1, sizeof(data), in)) > 0) {
        ok = fwrite(data, 1, n, out) == n;
    }
    if (in) {
        fclose(in);
    }
    if (out && fclose(out) != 0) {
        ok = false;
    }
    return ok;
}

static const char *const s_icons[] = {"icon_start.png", "icon_stop.png", "icon_poi_on.png", "icon_poi_off.png",
                                      "icon_config.png", "icon_shutdown.png"};

static bool WriteWidgetPanel(const T_ServiceOptions *options)
{
    char path[PATH_SIZE * 2];
    FILE *f;

    mkdir(options->widgetDir, 0755);
    for (size_t i = 0; i < sizeof(s_icons) / sizeof(s_icons[0]); i++) {
        char from[PATH_SIZE * 2];
        snprintf(from, sizeof(from), "%s/%s", options->widgetTemplateDir, s_icons[i]);
        snprintf(path, sizeof(path), "%s/%s", options->widgetDir, s_icons[i]);
        if (!CopyFile(from, path)) {
            fprintf(stderr, "[psdk_telemetry] widget icon %s: cannot copy to %s\n", from, path);
            return false;
        }
    }

    snprintf(path, sizeof(path), "%s/widget_config.json", options->widgetDir);
    f = fopen(path, "w");
    if (f == NULL) {
        fprintf(stderr, "[psdk_telemetry] cannot write %s: %s\n", path, strerror(errno));
        return false;
    }
    fprintf(f,
            "{\n"
            "  \"version\": {\"major\": 1, \"minor\": 0},\n"
            "  \"main_interface\": {\n"
            "    \"floating_window\": {\"is_enable\": true},\n"
            "    \"speaker\": {\"is_enable_tts\": false, \"is_enable_voice\": false},\n"
            "    \"widget_list\": [\n"
            "      {\"widget_index\": %d, \"widget_type\": \"button\", \"widget_name\": \"Start acquisition\",\n"
            "       \"icon_file_set\": {\"icon_file_name_selected\": \"icon_start.png\", \"icon_file_name_unselected\": \"icon_start.png\"}},\n"
            "      {\"widget_index\": %d, \"widget_type\": \"button\", \"widget_name\": \"Stop acquisition\",\n"
            "       \"icon_file_set\": {\"icon_file_name_selected\": \"icon_stop.png\", \"icon_file_name_unselected\": \"icon_stop.png\"}},\n"
            "      {\"widget_index\": %d, \"widget_type\": \"switch\", \"widget_name\": \"POI tracking\",\n"
            "       \"icon_file_set\": {\"icon_file_name_selected\": \"icon_poi_on.png\", \"icon_file_name_unselected\": \"icon_poi_off.png\"}},\n"
            "      {\"widget_index\": %d, \"widget_type\": \"list\", \"widget_name\": \"Configuration\", \"list_item\": [",
            WIDGET_START, WIDGET_STOP, WIDGET_POI, WIDGET_CONFIG);
    for (int i = 0; i < s_configCount; i++) {
        fprintf(f, "%s\n        {\"item_name\": \"%s\", \"icon_file_set\": {\"icon_file_name_selected\": \"icon_config.png\", "
                   "\"icon_file_name_unselected\": \"icon_config.png\"}}", i ? "," : "", s_configs[i]);
    }
    fprintf(f, "]},\n"
               "      {\"widget_index\": %d, \"widget_type\": \"button\", \"widget_name\": \"Shutdown Pi\",\n"
               "       \"icon_file_set\": {\"icon_file_name_selected\": \"icon_shutdown.png\", "
               "\"icon_file_name_unselected\": \"icon_shutdown.png\"}}\n", WIDGET_SHUTDOWN);
    fprintf(f, "    ]\n"
               "  },\n"
               "  \"config_interface\": {\n"
               "    \"text_input_box\": {\"widget_name\": \"TextInputBox\", \"placeholder_text\": \"\", \"is_enable\": false}\n"
               "  }\n"
               "}\n");
    if (fclose(f) != 0) {
        return false;
    }
    return true;
}

bool Service_StartWidgets(const T_ServiceOptions *options)
{
    T_DjiReturnCode rc;

    ListConfigs(options->configDir);
    if (!WriteWidgetPanel(options)) {
        return false;
    }
    rc = DjiWidget_Init();
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        rc = DjiWidget_RegDefaultUiConfigByDirPath(options->widgetDir);
    }
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        rc = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH, DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                            options->widgetDir);
    }
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        rc = DjiWidget_RegHandlerList(s_widgetHandlers, sizeof(s_widgetHandlers) / sizeof(s_widgetHandlers[0]));
    }
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        fprintf(stderr, "[psdk_telemetry] widget setup failed (error 0x%08llX)\n", (unsigned long long) rc);
        return false;
    }
    s_widgetsReady = true;
    fprintf(stderr, "[psdk_telemetry] widgets ready: %d configuration(s), default %s\n", s_configCount,
            s_configs[s_configIndex]);
    ShowOnRc("POLOCALC: waiting for telemd");
    return true;
}

/* ---------------------------------------------------------------------------------------------- start / stop */
bool Service_StartSockets(const T_ServiceOptions *options)
{
    snprintf(s_telemetryPath, sizeof(s_telemetryPath), "%s", options->telemetrySocket);
    snprintf(s_controlPath, sizeof(s_controlPath), "%s", options->controlSocket);
    s_stop = false;

    s_telemetryListen = ListenOn(s_telemetryPath);
    s_controlListen = ListenOn(s_controlPath);
    if (s_telemetryListen < 0 || s_controlListen < 0) {
        return false;
    }
    if (pthread_create(&s_telemetryThread, NULL, TelemetryAcceptThread, NULL) != 0 ||
        pthread_create(&s_controlThread, NULL, ControlThread, NULL) != 0) {
        return false;
    }
    s_telemetryRunning = s_controlRunning = true;
    pthread_setname_np(s_telemetryThread, "svc_telemetry");
    pthread_setname_np(s_controlThread, "svc_control");
    fprintf(stderr, "[psdk_telemetry] service: telemetry on %s, control on %s\n", s_telemetryPath, s_controlPath);
    return true;
}

void Service_Stop(void)
{
    s_stop = true;
    if (s_telemetryRunning) {
        pthread_join(s_telemetryThread, NULL);
        s_telemetryRunning = false;
    }
    if (s_controlRunning) {
        pthread_join(s_controlThread, NULL);
        s_controlRunning = false;
    }
    /* readers see the end of the stream, and the socket files go: lager and telemd notice at once */
    if (s_telemetryListen >= 0) {
        close(s_telemetryListen);
        unlink(s_telemetryPath);
        s_telemetryListen = -1;
    }
    if (s_controlListen >= 0) {
        close(s_controlListen);
        unlink(s_controlPath);
        s_controlListen = -1;
    }
    pthread_mutex_lock(&s_clientsLock);
    for (int i = 0; i < s_clientCount; i++) {
        close(s_clients[i]);
    }
    s_clientCount = 0;
    pthread_mutex_unlock(&s_clientsLock);
    pthread_mutex_lock(&s_controlLock);
    if (s_control >= 0) {
        close(s_control);
        s_control = -1;
    }
    pthread_mutex_unlock(&s_controlLock);
}
