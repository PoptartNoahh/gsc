/*
 * GSC CLI - encode, decode, and benchmark .gsc files.
 *
 * Usage:
 *   gsc encode  input.bin  output.gsc  [max_depth] [max_distance]
 *   gsc decode  input.gsc  output.bin
 *   gsc bench   input.bin  [max_depth] [max_distance]
 *
 * .bin format: flat float32 array, 4 values per point (x, y, z, ignored).
 * .gsc format: GSC2 binary blob (see gsc.h).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "gsc.h"

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

static int write_file(const char *path, const void *data, size_t len) {
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return -1; }
    fwrite(data, 1, len, f);
    fclose(f);
    return 0;
}

static int read_file(const char *path, uint8_t **out, size_t *out_len) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return -1; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    rewind(f);
    *out = malloc(sz);
    fread(*out, 1, sz, f);
    fclose(f);
    *out_len = (size_t)sz;
    return 0;
}

/* ------------------------------------------------------------------ */

static int cmd_encode(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "Usage: %s encode <input.bin> <output.gsc> [depth] [max_dist]\n", argv[0]);
        return 1;
    }
    int depth = argc > 4 ? atoi(argv[4]) : 8;
    float max_dist = argc > 5 ? (float)atof(argv[5]) : 200.0f;

    double *pts; int n;
    if (load_bin(argv[2], &pts, &n, max_dist)) return 1;

    gsc_config cfg = { .max_depth = (uint8_t)depth, .max_distance = max_dist };
    size_t enc_size;
    uint8_t *buf = gsc_encode(pts, (uint32_t)n, cfg, &enc_size, NULL);
    free(pts);
    if (!buf) { fprintf(stderr, "encode failed\n"); return 1; }

    if (write_file(argv[3], buf, enc_size)) { free(buf); return 1; }

    printf("%d points -> %zu bytes (%.2f:1)\n", n, enc_size, (double)(n*12) / enc_size);
    printf("wrote %s\n", argv[3]);

    free(buf);
    return 0;
}

static int cmd_decode(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "Usage: %s decode <input.gsc> <output.bin>\n", argv[0]);
        return 1;
    }

    uint8_t *data; size_t len;
    if (read_file(argv[2], &data, &len)) return 1;

    uint32_t n;
    double *pts = gsc_decode(data, len, &n);
    free(data);
    if (!pts) { fprintf(stderr, "decode failed\n"); return 1; }

    /* write as float32 x,y,z,0 to match .bin format */
    float *out = malloc((size_t)n * 4 * sizeof(float));
    for (uint32_t i = 0; i < n; i++) {
        out[i*4+0] = (float)pts[i*3+0];
        out[i*4+1] = (float)pts[i*3+1];
        out[i*4+2] = (float)pts[i*3+2];
        out[i*4+3] = 0.0f;
    }
    free(pts);

    if (write_file(argv[3], out, (size_t)n * 4 * sizeof(float))) { free(out); return 1; }

    printf("%u points -> %s\n", n, argv[3]);

    free(out);
    return 0;
}

static int cmd_bench(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s bench <input.bin> [depth] [max_dist]\n", argv[0]);
        return 1;
    }
    int depth = argc > 3 ? atoi(argv[3]) : 8;
    float max_dist = argc > 4 ? (float)atof(argv[4]) : 200.0f;

    double *pts; int n;
    if (load_bin(argv[2], &pts, &n, max_dist)) return 1;

    printf("Loaded %d points\n\n", n);

    double *recon = malloc((size_t)n * 3 * sizeof(double));
    gsc_config cfg = { .max_depth = (uint8_t)depth, .max_distance = max_dist };

    double t0 = now_sec();
    size_t enc_size;
    uint8_t *encoded = gsc_encode(pts, (uint32_t)n, cfg, &enc_size, recon);
    double enc_time = now_sec() - t0;
    if (!encoded) { fprintf(stderr, "encode failed\n"); return 1; }

    t0 = now_sec();
    uint32_t dec_n;
    double *decoded = gsc_decode(encoded, enc_size, &dec_n);
    double dec_time = now_sec() - t0;
    if (!decoded) { fprintf(stderr, "decode failed\n"); return 1; }

    int raw_size = n * 12;
    printf("  Raw float32:   %10d B  (%7.1f KB)\n", raw_size, raw_size/1024.0);
    printf("  GSC:           %10zu B  (%7.1f KB)\n", enc_size, enc_size/1024.0);
    printf("  Ratio:         %10.2f : 1\n", (double)raw_size / enc_size);
    printf("  Encode:        %10.3f s  (%.1fK pts/s)\n", enc_time, n/enc_time/1000);
    printf("  Decode:        %10.3f s  (%.1fK pts/s)\n", dec_time, dec_n/dec_time/1000);
    printf("  Points out:    %10u\n\n", dec_n);

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

    printf("  Ang error:     mean %.4f deg  max %.4f deg\n", ang_sum/n, ang_max);
    printf("  Dist error:    mean %.2f mm   max %.2f mm\n", dst_sum/n*1000, dst_max*1000);

    free(pts);
    free(recon);
    free(encoded);
    free(decoded);
    return 0;
}

/* ------------------------------------------------------------------ */

int main(int argc, char **argv) {
    if (argc < 2) goto usage;

    if (strcmp(argv[1], "encode") == 0) return cmd_encode(argc, argv);
    if (strcmp(argv[1], "decode") == 0) return cmd_decode(argc, argv);
    if (strcmp(argv[1], "bench")  == 0) return cmd_bench(argc, argv);

usage:
    fprintf(stderr,
        "Usage:\n"
        "  %s encode <input.bin> <output.gsc> [depth] [max_dist]\n"
        "  %s decode <input.gsc> <output.bin>\n"
        "  %s bench  <input.bin> [depth] [max_dist]\n",
        argv[0], argv[0], argv[0]);
    return 1;
}
