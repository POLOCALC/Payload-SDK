/**
 * telemetry_stream.h - framed binary stream of PSDK telemetry records (POLOCALC).
 *
 * Every record, little endian, packed:
 *
 *   offset  size  field
 *        0     2  sync: 0xA5 0x5A
 *        2     1  format version (TELEMETRY_STREAM_VERSION)
 *        3     1  record type (E_StreamRecordType)
 *        4     2  topic id: E_DjiFcSubscriptionTopic for DATA records, 0 otherwise
 *        6     2  payload length N
 *        8     8  host time: CLOCK_REALTIME, nanoseconds since the Unix epoch (int64)
 *       16     4  aircraft timestamp, millisecond part (0 if not a DATA record)
 *       20     4  aircraft timestamp, microsecond part (0 if not a DATA record)
 *       24     N  payload: the raw, packed PSDK struct for DATA records
 *     24+N     2  CRC-16/CCITT-FALSE (init 0xFFFF, poly 0x1021) of bytes [2, 24+N)
 *
 * Payloads of the other record types:
 *   STATUS:      u8 state (E_StreamState), then a UTF-8 message (not NUL-terminated)
 *   TOPIC_TABLE: u16 count, then count x { u16 topic, u16 frequency_hz, u16 size, u8 subscribed, char name[32] }
 *   STATS:       u32 dropped_records, u16 count, then count x { u16 topic, u32 received }
 */
#ifndef TELEMETRY_STREAM_H
#define TELEMETRY_STREAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_STREAM_VERSION        1
#define TELEMETRY_STREAM_SYNC0          0xA5
#define TELEMETRY_STREAM_SYNC1          0x5A
#define TELEMETRY_STREAM_HEADER_SIZE    24
#define TELEMETRY_STREAM_CRC_SIZE       2
#define TELEMETRY_STREAM_MAX_PAYLOAD    2048    /* the TOPIC_TABLE record is 2 + 39 bytes per topic */
#define TELEMETRY_STREAM_NAME_SIZE      32

typedef enum {
    STREAM_RECORD_DATA        = 1,
    STREAM_RECORD_STATUS      = 2,
    STREAM_RECORD_TOPIC_TABLE = 3,
    STREAM_RECORD_STATS       = 4,
} E_StreamRecordType;

typedef enum {
    STREAM_STATE_STARTING      = 1,   /* program started, preparing the platform */
    STREAM_STATE_CONNECTING    = 2,   /* waiting for the aircraft (DjiCore_Init) */
    STREAM_STATE_CONNECTED     = 3,   /* connected and authenticated */
    STREAM_STATE_SUBSCRIBED    = 4,   /* topics subscribed, data is flowing */
    STREAM_STATE_STOPPING      = 5,   /* clean shutdown in progress */
    STREAM_STATE_INFO          = 10,  /* informational message */
    STREAM_STATE_WARNING       = 11,  /* non-fatal problem, e.g. one topic not supported */
    STREAM_STATE_ERROR         = 100, /* fatal error, the program exits */
} E_StreamState;

/* Open the output (a file descriptor already open for writing) and start the writer thread. */
bool TelemetryStream_Start(int outputFd);

/* Called by the writer thread with each complete record (header, payload and CRC). */
typedef void (*TelemetryStreamSink)(const uint8_t *record, size_t length);

/* Start the writer thread with a sink instead of a file descriptor (service mode: several clients). */
bool TelemetryStream_StartWithSink(TelemetryStreamSink sink);

/* Flush everything still queued and stop the writer thread. */
void TelemetryStream_Stop(void);

/* Queue a record. Safe to call from any thread, never blocks: if the queue is full the record is dropped. */
void TelemetryStream_Put(E_StreamRecordType type, uint16_t topic, const void *payload, uint16_t length,
                         uint32_t aircraftMs, uint32_t aircraftUs);

/* Convenience: queue a STATUS record with a printf-style message (also logged to stderr). */
void TelemetryStream_Status(E_StreamState state, const char *format, ...);

/* Number of records dropped because the queue was full. */
uint32_t TelemetryStream_Dropped(void);

/* True if the reader went away (broken pipe) or a write failed: the program should exit. */
bool TelemetryStream_Failed(void);

#endif /* TELEMETRY_STREAM_H */
