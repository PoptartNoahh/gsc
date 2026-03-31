/*
 * GSC3 Streaming Benchmark
 *
 * Tests:
 *   1. Parallel encode  (all faces at once, pthreads)
 *   2. Parallel decode   (all chunks at once, pthreads)
 *   3. Sequential chunk decode (one at a time, simulates receiving chunks)
 *   4. Streaming pipeline (producer-consumer with overlapped encode/decode)
 *   5. Raw float32 baseline (memcpy throughput)
 *   6. Simulated network transfer at various bandwidths
 *   7. Error analysis
 *
 * Usage: stream_bench <input.bin> [depth] [max_dist]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include "gsc3.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static int load_bin(const char *path, double **out_pts, int *out_n, float max_dist) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return -1; }
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    rewind(f);

    int total = (int)(fsize / (4 * sizeof(float)));
    float *raw = malloc(fsize);
    fread(raw, 1, fsize, f);
    fclose(f);

    double *pts = malloc((size_t)total * 3 * sizeof(double));
    int n = 0;
    for (int i = 0; i < total; i++) {
        double x = raw[i*4], y = raw[i*4+1], z = raw[i*4+2];
        double d = sqrt(x*x + y*y + z*z);
        if (d > 0.5 && d < (double)max_dist) {
            pts[n*3]   = x;
            pts[n*3+1] = y;
            pts[n*3+2] = z;
            n++;
        }
    }
    free(raw);
    *out_pts = pts;
    *out_n = n;
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Streaming pipeline: producer-consumer via single-slot channel      */
/* ------------------------------------------------------------------ */

typedef struct {
    pthread_mutex_t mtx;
    pthread_cond_t  ready;
    pthread_cond_t  consumed;
    gsc3_chunk      slot;
    int             has_chunk;
    int             finished;
} channel;

typedef struct {
    const double  *points;
    uint32_t       n;
    gsc3_config    cfg;
    channel       *ch;
} producer_args;

static void *producer_fn(void *arg) {
    producer_args *pa = arg;
    gsc3_enc_ctx *enc = gsc3_enc_begin(pa->points, pa->n, pa->cfg, NULL);
    if (!enc) return NULL;

    gsc3_chunk chunk;
    while (gsc3_enc_next(enc, &chunk) == 0) {
        pthread_mutex_lock(&pa->ch->mtx);
        while (pa->ch->has_chunk)
            pthread_cond_wait(&pa->ch->consumed, &pa->ch->mtx);
        pa->ch->slot = chunk;
        pa->ch->has_chunk = 1;
        pthread_cond_signal(&pa->ch->ready);
        pthread_mutex_unlock(&pa->ch->mtx);
    }

    pthread_mutex_lock(&pa->ch->mtx);
    pa->ch->finished = 1;
    pthread_cond_signal(&pa->ch->ready);
    pthread_mutex_unlock(&pa->ch->mtx);

    gsc3_enc_free(enc);
    return NULL;
}

/* ------------------------------------------------------------------ */
/*  Main benchmark                                                     */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <input.bin> [depth] [max_dist]\n", argv[0]);
        return 1;
    }

    int depth     = argc > 2 ? atoi(argv[2]) : 8;
    float max_dist = argc > 3 ? (float)atof(argv[3]) : 200.0f;

    double *pts; int n;
    if (load_bin(argv[1], &pts, &n, max_dist)) return 1;

    size_t raw_bytes = (size_t)n * 3 * sizeof(double);
    size_t raw_f32   = (size_t)n * 12;  /* float32 xyz, 4 bytes each */

    gsc3_config cfg = { .max_depth = (uint8_t)depth, .max_distance = max_dist, .threads = 0 };

    printf("GSC3 Streaming Benchmark\n");
    printf("========================\n\n");
    printf("Input:   %s\n", argv[1]);
    printf("Points:  %d  |  Raw f32 XYZ: %.0f KB\n\n", n, raw_f32 / 1024.0);

    /* ---- 1. Parallel Encode ---- */
    double *recon = malloc(raw_bytes);
    gsc3_stream stream;

    double t0 = now_sec();
    gsc3_encode(pts, (uint32_t)n, cfg, &stream, recon);
    double enc_time = now_sec() - t0;

    size_t wire_len;
    uint8_t *wire = gsc3_serialize(&stream, &wire_len);

    printf("Parallel Encode (%d active faces)\n", stream.num_chunks);
    printf("  time      %.4f s\n", enc_time);
    printf("  speed     %.1fM pts/s\n", n / enc_time / 1e6);
    printf("  wire      %.0f KB (%.1f:1 vs raw f32)\n\n",
           wire_len / 1024.0, (double)raw_f32 / wire_len);

    /* ---- 2. Parallel Decode ---- */
    uint32_t dec_n;
    t0 = now_sec();
    double *dec_pts = gsc3_decode(&stream, &dec_n);
    double dec_time = now_sec() - t0;

    printf("Parallel Decode (%d threads)\n", stream.num_chunks);
    printf("  time      %.4f s\n", dec_time);
    printf("  speed     %.1fM pts/s\n", dec_n / dec_time / 1e6);
    printf("  points    %u\n\n", dec_n);
    free(dec_pts);

    /* ---- 3. Sequential Chunk Decode ---- */
    double first_chunk_time = 0;
    t0 = now_sec();
    for (int i = 0; i < stream.num_chunks; i++) {
        uint32_t cn;
        double *cp = gsc3_decode_chunk(&stream.chunks[i],
                                       stream.max_depth,
                                       stream.max_distance, &cn);
        if (i == 0) first_chunk_time = now_sec() - t0;
        free(cp);
    }
    double seq_time = now_sec() - t0;

    printf("Sequential Chunk Decode\n");
    printf("  chunks    %d\n", stream.num_chunks);
    printf("  first pt  %.2f ms\n", first_chunk_time * 1000);
    printf("  total     %.4f s\n\n", seq_time);

    /* ---- 4. Streaming Pipeline (producer-consumer) ---- */
    channel ch;
    pthread_mutex_init(&ch.mtx, NULL);
    pthread_cond_init(&ch.ready, NULL);
    pthread_cond_init(&ch.consumed, NULL);
    ch.has_chunk = 0;
    ch.finished  = 0;

    producer_args pa = { .points = pts, .n = (uint32_t)n, .cfg = cfg, .ch = &ch };
    pthread_t prod_thread;

    int pipe_chunks = 0;
    uint32_t pipe_points = 0;
    double pipe_first = 0;

    t0 = now_sec();
    pthread_create(&prod_thread, NULL, producer_fn, &pa);

    for (;;) {
        pthread_mutex_lock(&ch.mtx);
        while (!ch.has_chunk && !ch.finished)
            pthread_cond_wait(&ch.ready, &ch.mtx);
        if (!ch.has_chunk && ch.finished) {
            pthread_mutex_unlock(&ch.mtx);
            break;
        }
        gsc3_chunk chunk = ch.slot;
        ch.has_chunk = 0;
        pthread_cond_signal(&ch.consumed);
        pthread_mutex_unlock(&ch.mtx);

        uint32_t cn;
        double *cp = gsc3_decode_chunk(&chunk, cfg.max_depth,
                                       (uint16_t)(int)cfg.max_distance, &cn);
        pipe_chunks++;
        pipe_points += cn;
        if (pipe_chunks == 1) pipe_first = now_sec() - t0;
        free(cp);
        free(chunk.data);
    }

    pthread_join(prod_thread, NULL);
    double pipe_time = now_sec() - t0;

    pthread_mutex_destroy(&ch.mtx);
    pthread_cond_destroy(&ch.ready);
    pthread_cond_destroy(&ch.consumed);

    printf("Streaming Pipeline (encode + decode overlapped)\n");
    printf("  chunks    %d\n", pipe_chunks);
    printf("  first pt  %.2f ms\n", pipe_first * 1000);
    printf("  total     %.4f s\n", pipe_time);
    printf("  speed     %.1fM pts/s\n\n", pipe_points / pipe_time / 1e6);

    /* ---- 5. Raw float32 Baseline ---- */
    float *raw_buf = malloc(raw_f32);
    for (int i = 0; i < n; i++) {
        raw_buf[i*3+0] = (float)pts[i*3+0];
        raw_buf[i*3+1] = (float)pts[i*3+1];
        raw_buf[i*3+2] = (float)pts[i*3+2];
    }
    float *raw_copy = malloc(raw_f32);

    volatile float sink = 0;
    int iters = 1000;
    t0 = now_sec();
    for (int it = 0; it < iters; it++) {
        memcpy(raw_copy, raw_buf, raw_f32);
        sink += raw_copy[it % n];
    }
    double raw_time = (now_sec() - t0) / iters;
    (void)sink;

    printf("Raw float32 Baseline\n");
    printf("  wire      %.0f KB\n", raw_f32 / 1024.0);
    printf("  memcpy    %.6f s (%.1f GB/s)\n\n", raw_time, raw_f32 / raw_time / 1e9);

    free(raw_buf);
    free(raw_copy);

    /* ---- 6. Simulated Network Transfer ---- */
    double bws[] = { 1e6, 10e6, 100e6, 1e9 };
    const char *bw_labels[] = { "1 Mbps", "10 Mbps", "100 Mbps", "1 Gbps" };
    int nbw = 4;

    printf("Network Transfer Simulation\n");
    printf("  %-12s  %-18s  %-18s  %s\n", "bandwidth", "GSC3 (xfer+dec)", "Raw (xfer only)", "speedup");
    for (int i = 0; i < nbw; i++) {
        double gsc_xfer = (double)wire_len * 8.0 / bws[i];
        double gsc_total = gsc_xfer + dec_time;
        double raw_xfer  = (double)raw_f32 * 8.0 / bws[i];
        printf("  %-12s  %7.3f + %-7.3f s  %12.3f s    %.1fx\n",
               bw_labels[i], gsc_xfer, dec_time, raw_xfer, raw_xfer / gsc_total);
    }
    printf("\n");

    /* ---- 7. Error Analysis ---- */
    double ang_sum = 0, ang_max = 0;
    double dst_sum = 0, dst_max = 0;
    for (int i = 0; i < n; i++) {
        double ox = pts[i*3], oy = pts[i*3+1], oz = pts[i*3+2];
        double rx = recon[i*3], ry = recon[i*3+1], rz = recon[i*3+2];
        double od = sqrt(ox*ox + oy*oy + oz*oz);
        double rd = sqrt(rx*rx + ry*ry + rz*rz);

        double de = fabs(od - rd);
        dst_sum += de;
        if (de > dst_max) dst_max = de;

        double dot = (ox*rx + oy*ry + oz*rz);
        double den = od * rd;
        if (den > 1e-30) dot /= den;
        if (dot >  1) dot =  1;
        if (dot < -1) dot = -1;
        double ae = acos(dot) * (180.0 / M_PI);
        ang_sum += ae;
        if (ae > ang_max) ang_max = ae;
    }

    printf("Error (vs input)\n");
    printf("  angular   mean %.4f deg   max %.4f deg\n", ang_sum/n, ang_max);
    printf("  distance  mean %.2f mm    max %.2f mm\n", dst_sum/n*1000, dst_max*1000);

    /* cleanup */
    free(pts);
    free(recon);
    free(wire);
    gsc3_stream_free(&stream);
    return 0;
}
