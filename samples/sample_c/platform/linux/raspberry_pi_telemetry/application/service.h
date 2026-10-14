/**
 * service.h - psdk_telemetry service mode (--service): the program runs at boot as psdk-bridge.service, owns the
 * PSDK connection and shares it:
 *  - telemetry socket: the binary stream (telemetry_stream.h) to any number of readers (lager's DJI_M400 module).
 *    A reader that connects gets the CONNECTED status and the TOPIC_TABLE first, then the live records.
 *  - control socket: one text client, telemd. Line protocol, one command per line:
 *      service -> telemd:  "CMD start <config>", "CMD stop", "CMD poi on", "CMD poi off", "CMD shutdown"
 *      telemd -> service:  "TEXT <status line>"     shown in the RC's floating window
 *                          "MSG <reply>"            shown for a few seconds instead of the status line
 *                          "STATE <running> <poi>"  0/1 each: porter running, POI tracking active
 *  - widgets on the main panel of DJI Pilot 2: Start / Stop acquisition buttons, a POI tracking switch, the list of
 *    porter configurations (the .yml files of config/) and a Shutdown Pi button (press it twice within 5 s).
 */
#ifndef PSDK_SERVICE_H
#define PSDK_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    const char *telemetrySocket;    /* path of the telemetry socket */
    const char *controlSocket;      /* path of the control socket */
    const char *configDir;          /* porter's config/ folder (list of configurations), or NULL */
    const char *widgetTemplateDir;  /* folder with the widget icons */
    const char *widgetDir;          /* where the widget panel (widget_config.json + icons) is generated */
} T_ServiceOptions;

/* Stream sink (TelemetryStream_StartWithSink): sends each record to every telemetry client. */
void Service_StreamSink(const uint8_t *record, size_t length);

/* Generate and register the widget panel. Call after DjiCore_Init and before DjiCore_ApplicationStart. */
bool Service_StartWidgets(const T_ServiceOptions *options);

/* Open the telemetry and control sockets. Call once the topics are subscribed. */
bool Service_StartSockets(const T_ServiceOptions *options);

/* Close the sockets and remove their files. */
void Service_Stop(void);

#endif /* PSDK_SERVICE_H */
