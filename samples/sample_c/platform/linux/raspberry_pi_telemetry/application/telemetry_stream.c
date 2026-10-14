/**
 * telemetry_stream.c - framed binary stream of PSDK telemetry records (POLOCALC).
 *
 * The PSDK delivers subscription data in its own threads. Those callbacks must return quickly, so they only
 * encode the record into a fixed-size ring buffer; a writer thread sends the queued records to the output.
 * If the reader is slower than the data, the newest records are dropped and counted, the SDK is never blocked.
 * See telemetry_stream.h for the record format.
 */
#define _GNU_SOURCE                 /* pthread_setname_np() */
#include "telemetry_stream.h"

#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define RING_SLOTS        1024
#define SLOT_SIZE         (TELEMETRY_STREAM_HEADER_SIZE + TELEMETRY_STREAM_MAX_PAYLOAD + TELEMETRY_STREAM_CRC_SIZE)

typedef struct {
    uint16_t length;
    uint8_t  bytes[SLOT_SIZE];
} T_Slot;

static T_Slot          s_ring[RING_SLOTS];
static unsigned        s_head;          /* next slot to write */
static unsigned        s_tail;          /* next slot to send */
static unsigned        s_used;
static uint32_t        s_dropped;
static bool            s_stop;
static volatile bool   s_failed;
static bool            s_running;
static int             s_fd = -1;
static TelemetryStreamSink s_sink;      /* if set, records go to it instead of s_fd (service mode) */
static pthread_t       s_thread;
static pthread_mutex_t s_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  s_cond = PTHREAD_COND_INITIALIZER;

static uint16_t Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t) data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (uint16_t) ((crc << 1) ^ 0x1021) : (uint16_t) (crc << 1);
        }
    }
    return crc;
}

static void PutU16(uint8_t *p, uint16_t v) { p[0] = (uint8_t) v; p[1] = (uint8_t) (v >> 8); }

static void PutU32(uint8_t *p, uint32_t v)
{
    for (int i = 0; i < 4; i++) {
        p[i] = (uint8_t) (v >> (8 * i));
    }
}

static void PutI64(uint8_t *p, int64_t value)
{
    uint64_t v = (uint64_t) value;
    for (int i = 0; i < 8; i++) {
        p[i] = (uint8_t) (v >> (8 * i));
    }
}

static bool WriteAll(int fd, const uint8_t *data, size_t length)
{
    while (length > 0) {
        ssize_t n = write(fd, data, length);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;                       /* EPIPE: the reader went away */
        }
        data += n;
        length -= (size_t) n;
    }
    return true;
}

static void *WriterThread(void *argument)
{
    static uint8_t out[SLOT_SIZE];
    (void) argument;

    for (;;) {
        uint16_t length;

        pthread_mutex_lock(&s_lock);
        while (s_used == 0 && !s_stop) {
            pthread_cond_wait(&s_cond, &s_lock);
        }
        if (s_used == 0 && s_stop) {            /* everything sent: done */
            pthread_mutex_unlock(&s_lock);
            break;
        }
        length = s_ring[s_tail].length;
        memcpy(out, s_ring[s_tail].bytes, length);
        s_tail = (s_tail + 1) % RING_SLOTS;
        s_used--;
        pthread_mutex_unlock(&s_lock);

        if (s_sink != NULL) {
            s_sink(out, length);
        } else if (!s_failed && !WriteAll(s_fd, out, length)) {
            s_failed = true;
            fprintf(stderr, "[psdk_telemetry] output write failed: %s\n", strerror(errno));
        }
    }
    return NULL;
}

bool TelemetryStream_StartWithSink(TelemetryStreamSink sink)
{
    s_sink = sink;
    return TelemetryStream_Start(-1);
}

bool TelemetryStream_Start(int outputFd)
{
    s_fd = outputFd;
    s_stop = false;
    if (pthread_create(&s_thread, NULL, WriterThread, NULL) != 0) {
        return false;
    }
    pthread_setname_np(s_thread, "stream_writer");
    s_running = true;
    return true;
}

void TelemetryStream_Stop(void)
{
    if (!s_running) {
        return;
    }
    pthread_mutex_lock(&s_lock);
    s_stop = true;
    pthread_cond_signal(&s_cond);
    pthread_mutex_unlock(&s_lock);
    pthread_join(s_thread, NULL);
    s_running = false;
}

void TelemetryStream_Put(E_StreamRecordType type, uint16_t topic, const void *payload, uint16_t length,
                         uint32_t aircraftMs, uint32_t aircraftUs)
{
    struct timespec now;
    uint8_t *p;

    if (length > TELEMETRY_STREAM_MAX_PAYLOAD) {
        length = TELEMETRY_STREAM_MAX_PAYLOAD;
    }
    clock_gettime(CLOCK_REALTIME, &now);

    pthread_mutex_lock(&s_lock);
    if (s_used == RING_SLOTS) {                 /* queue full: drop this record, never block the SDK */
        s_dropped++;
        pthread_mutex_unlock(&s_lock);
        return;
    }
    p = s_ring[s_head].bytes;
    p[0] = TELEMETRY_STREAM_SYNC0;
    p[1] = TELEMETRY_STREAM_SYNC1;
    p[2] = TELEMETRY_STREAM_VERSION;
    p[3] = (uint8_t) type;
    PutU16(p + 4, topic);
    PutU16(p + 6, length);
    PutI64(p + 8, (int64_t) now.tv_sec * 1000000000LL + now.tv_nsec);
    PutU32(p + 16, aircraftMs);
    PutU32(p + 20, aircraftUs);
    if (length > 0) {
        memcpy(p + TELEMETRY_STREAM_HEADER_SIZE, payload, length);
    }
    PutU16(p + TELEMETRY_STREAM_HEADER_SIZE + length,
           Crc16(p + 2, TELEMETRY_STREAM_HEADER_SIZE - 2 + length));
    s_ring[s_head].length = TELEMETRY_STREAM_HEADER_SIZE + length + TELEMETRY_STREAM_CRC_SIZE;
    s_head = (s_head + 1) % RING_SLOTS;
    s_used++;
    pthread_cond_signal(&s_cond);
    pthread_mutex_unlock(&s_lock);
}

void TelemetryStream_Status(E_StreamState state, const char *format, ...)
{
    uint8_t payload[1 + 255];
    va_list args;
    int n;

    va_start(args, format);
    n = vsnprintf((char *) payload + 1, sizeof(payload) - 1, format, args);
    va_end(args);
    if (n < 0) {
        n = 0;
    } else if (n > (int) sizeof(payload) - 2) {
        n = (int) sizeof(payload) - 2;
    }
    payload[0] = (uint8_t) state;

    fprintf(stderr, "[psdk_telemetry] %s%.*s\n",
            state == STREAM_STATE_ERROR ? "ERROR: " : state == STREAM_STATE_WARNING ? "WARNING: " : "",
            n, (char *) payload + 1);
    TelemetryStream_Put(STREAM_RECORD_STATUS, 0, payload, (uint16_t) (1 + n), 0, 0);
}

uint32_t TelemetryStream_Dropped(void)
{
    uint32_t dropped;
    pthread_mutex_lock(&s_lock);
    dropped = s_dropped;
    pthread_mutex_unlock(&s_lock);
    return dropped;
}

bool TelemetryStream_Failed(void)
{
    return s_failed;
}
