/*
 * GSC v3 - streaming + parallel implementation.
 *
 * Core algorithm identical to GSC v2 (gsc.c).  Internals are duplicated
 * here so v2 is completely untouched - both compile independently.
 *
 * New in v3:
 *   - each icosahedron face is an independent chunk
 *   - parallel encode/decode via pthreads (up to 20-way)
 *   - streaming encoder: produce one chunk at a time
 *   - chunk-level decode: consume chunks as they arrive
 */

#include "gsc3.h"

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <zlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ICO_V 12
#define ICO_F 20

/* ================================================================== */
/*  Duplicated internals from gsc.c (all static, no symbol conflicts) */
/* ================================================================== */

typedef struct { double x, y, z; } v3;

static inline v3  v3_add(v3 a, v3 b)      { return (v3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline v3  v3_sub(v3 a, v3 b)      { return (v3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline v3  v3_scale(v3 a, double s) { return (v3){a.x*s, a.y*s, a.z*s}; }
static inline double v3_dot(v3 a, v3 b)   { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline v3 v3_cross(v3 a, v3 b) {
    return (v3){ a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
static inline double v3_len(v3 a) { return sqrt(v3_dot(a, a)); }
static inline v3 v3_norm(v3 a) {
    double n = v3_len(a);
    return n > 1e-15 ? v3_scale(a, 1.0/n) : a;
}
static inline v3 v3_centroid(v3 a, v3 b, v3 c) {
    return v3_norm(v3_add(v3_add(a, b), c));
}

typedef struct {
    v3  verts[ICO_V];
    int faces[ICO_F][3];
} icosahedron;

static void ico_init(icosahedron *ico) {
    v3 *V = ico->verts;
    V[0]  = (v3){0, 0, 1};
    V[11] = (v3){0, 0,-1};

    double zr = 1.0 / sqrt(5.0);
    double rr = 2.0 / sqrt(5.0);
    for (int k = 0; k < 5; k++) {
        double a = k * 2.0 * M_PI / 5.0;
        V[1+k] = (v3){rr*cos(a), rr*sin(a), zr};
    }
    for (int k = 0; k < 5; k++) {
        double a = (k + 0.5) * 2.0 * M_PI / 5.0;
        V[6+k] = (v3){rr*cos(a), rr*sin(a), -zr};
    }

    int fi = 0;
    for (int k=0;k<5;k++) { ico->faces[fi][0]=0; ico->faces[fi][1]=1+k; ico->faces[fi][2]=1+(k+1)%5; fi++; }
    for (int k=0;k<5;k++) { ico->faces[fi][0]=1+k; ico->faces[fi][1]=6+k; ico->faces[fi][2]=1+(k+1)%5; fi++; }
    for (int k=0;k<5;k++) { ico->faces[fi][0]=6+k; ico->faces[fi][1]=1+(k+1)%5; ico->faces[fi][2]=6+(k+1)%5; fi++; }
    for (int k=0;k<5;k++) { ico->faces[fi][0]=11; ico->faces[fi][1]=6+(k+1)%5; ico->faces[fi][2]=6+k; fi++; }

    for (int i = 0; i < ICO_F; i++) {
        v3 v0 = V[ico->faces[i][0]];
        v3 v1 = V[ico->faces[i][1]];
        v3 v2 = V[ico->faces[i][2]];
        v3 cent = v3_add(v3_add(v0, v1), v2);
        v3 nrm  = v3_cross(v3_sub(v1, v0), v3_sub(v2, v0));
        if (v3_dot(nrm, cent) < 0) {
            int tmp = ico->faces[i][1];
            ico->faces[i][1] = ico->faces[i][2];
            ico->faces[i][2] = tmp;
        }
    }
}

typedef struct { v3 v[3]; } tri;

static void subdivide(v3 v0, v3 v1, v3 v2, tri out[4]) {
    v3 m01 = v3_norm(v3_scale(v3_add(v0, v1), 0.5));
    v3 m12 = v3_norm(v3_scale(v3_add(v1, v2), 0.5));
    v3 m20 = v3_norm(v3_scale(v3_add(v2, v0), 0.5));
    out[0] = (tri){{v0,  m01, m20}};
    out[1] = (tri){{m01, v1,  m12}};
    out[2] = (tri){{m20, m12, v2 }};
    out[3] = (tri){{m01, m12, m20}};
}

/* -- Bit-level I/O -- */

typedef struct {
    uint8_t *buf;
    size_t   cap, len;
    uint8_t  cur;
    int      pos;
} bwriter;

static void bw_init(bwriter *w, size_t cap) {
    if (cap < 256) cap = 256;
    w->buf = malloc(cap);
    w->cap = cap; w->len = 0; w->cur = 0; w->pos = 0;
}

static void bw_bit(bwriter *w, int bit) {
    w->cur = (w->cur << 1) | (bit & 1);
    if (++w->pos == 8) {
        if (w->len >= w->cap) { w->cap *= 2; w->buf = realloc(w->buf, w->cap); }
        w->buf[w->len++] = w->cur;
        w->cur = 0; w->pos = 0;
    }
}

static void bw_bits(bwriter *w, uint32_t val, int n) {
    for (int i = n-1; i >= 0; i--) bw_bit(w, (val >> i) & 1);
}

static void bw_flush(bwriter *w) {
    if (w->pos > 0) {
        w->cur <<= (8 - w->pos);
        if (w->len >= w->cap) { w->cap *= 2; w->buf = realloc(w->buf, w->cap); }
        w->buf[w->len++] = w->cur;
        w->cur = 0; w->pos = 0;
    }
}

typedef struct {
    const uint8_t *data;
    size_t len, byte;
    int    bit;
} breader;

static void br_init(breader *r, const uint8_t *data, size_t len) {
    r->data = data; r->len = len; r->byte = 0; r->bit = 0;
}

static int br_bit(breader *r) {
    if (r->byte >= r->len) return 0;
    int b = (r->data[r->byte] >> (7 - r->bit)) & 1;
    if (++r->bit == 8) { r->byte++; r->bit = 0; }
    return b;
}

static uint32_t br_bits(breader *r, int n) {
    uint32_t v = 0;
    for (int i = 0; i < n; i++) v = (v << 1) | br_bit(r);
    return v;
}

/* -- Classification -- */

static int point_in_tri(v3 dir, v3 v0, v3 v1, v3 v2) {
    return v3_dot(dir, v3_cross(v0, v1)) >= 0 &&
           v3_dot(dir, v3_cross(v1, v2)) >= 0 &&
           v3_dot(dir, v3_cross(v2, v0)) >= 0;
}

static int classify_face(v3 dir, const icosahedron *ico) {
    for (int fi = 0; fi < ICO_F; fi++) {
        v3 v0 = ico->verts[ico->faces[fi][0]];
        v3 v1 = ico->verts[ico->faces[fi][1]];
        v3 v2 = ico->verts[ico->faces[fi][2]];
        if (point_in_tri(dir, v0, v1, v2)) return fi;
    }
    double best = -2; int best_fi = 0;
    for (int fi = 0; fi < ICO_F; fi++) {
        v3 c = v3_centroid(ico->verts[ico->faces[fi][0]],
                           ico->verts[ico->faces[fi][1]],
                           ico->verts[ico->faces[fi][2]]);
        double d = v3_dot(dir, c);
        if (d > best) { best = d; best_fi = fi; }
    }
    return best_fi;
}

static int classify_sub(v3 dir, const tri subs[4]) {
    for (int i = 0; i < 4; i++)
        if (point_in_tri(dir, subs[i].v[0], subs[i].v[1], subs[i].v[2]))
            return i;
    double best = -2; int best_i = 0;
    for (int i = 0; i < 4; i++) {
        v3 c = v3_centroid(subs[i].v[0], subs[i].v[1], subs[i].v[2]);
        double d = v3_dot(dir, c);
        if (d > best) { best = d; best_i = i; }
    }
    return best_i;
}

/* -- Quantization -- */

static inline uint16_t quantize(double dist, double max_d) {
    double q = dist / max_d * 65535.0;
    if (q < 0)     q = 0;
    if (q > 65535) q = 65535;
    return (uint16_t)q;
}

static inline double dequantize(uint16_t q, double max_d) {
    return (double)q / 65535.0 * max_d;
}

static inline uint32_t zigzag_enc(int32_t v) {
    return ((uint32_t)v << 1) ^ (uint32_t)(v >> 31);
}

static inline int32_t zigzag_dec(uint32_t z) {
    return (int32_t)(z >> 1) ^ -(int32_t)(z & 1);
}

/* -- Subtree encode -- */

static void enc_subtree(bwriter *tw, bwriter *pw,
                        v3 v0, v3 v1, v3 v2,
                        const v3 *dirs, const uint16_t *qdist,
                        int *idx, int n,
                        double *recon, double max_d,
                        int depth, int max_depth, int *prev,
                        int *scratch)
{
    assert(n > 0);

    if (n == 1 ||
        (depth >= max_depth && n <= 255) ||
        depth >= max_depth + 8) {
        bw_bit(tw, 0);
        int count = n <= 255 ? n : 255;
        if (depth >= max_depth) bw_bits(tw, (uint32_t)count, 8);

        v3 c = v3_centroid(v0, v1, v2);
        for (int i = 0; i < count; i++) {
            int pi = idx[i];
            int qd = qdist[pi];
            int32_t delta = qd - *prev;
            uint32_t z = zigzag_enc(delta);
            if (z < 256) { bw_bit(pw, 0); bw_bits(pw, z, 8); }
            else          { bw_bit(pw, 1); bw_bits(pw, (uint32_t)qd, 16); }
            *prev = qd;

            if (recon) {
                double d = dequantize((uint16_t)qd, max_d);
                recon[pi*3+0] = c.x * d;
                recon[pi*3+1] = c.y * d;
                recon[pi*3+2] = c.z * d;
            }
        }
        return;
    }

    bw_bit(tw, 1);

    tri subs[4];
    subdivide(v0, v1, v2, subs);

    int counts[4] = {0};
    for (int i = 0; i < n; i++) {
        scratch[i] = classify_sub(dirs[idx[i]], subs);
        counts[scratch[i]]++;
    }

    uint32_t mask = 0;
    for (int c = 0; c < 4; c++)
        if (counts[c]) mask |= 1u << (3 - c);
    bw_bits(tw, mask, 4);

    int off[4], pos[4];
    off[0] = 0;
    for (int c = 1; c < 4; c++) off[c] = off[c-1] + counts[c-1];
    memcpy(pos, off, sizeof(off));

    int *tmp = scratch + n;
    for (int i = 0; i < n; i++)
        tmp[pos[scratch[i]]++] = idx[i];
    memcpy(idx, tmp, n * sizeof(int));

    for (int c = 0; c < 4; c++) {
        if (counts[c])
            enc_subtree(tw, pw, subs[c].v[0], subs[c].v[1], subs[c].v[2],
                        dirs, qdist, idx + off[c], counts[c],
                        recon, max_d, depth+1, max_depth, prev, scratch);
    }
}

/* -- Subtree decode -- */

static void dec_subtree(breader *tr, breader *pr,
                        v3 v0, v3 v1, v3 v2,
                        int depth, int max_depth, double max_d,
                        int *prev, double *out, int *pos)
{
    if (!br_bit(tr)) {
        v3 c = v3_centroid(v0, v1, v2);
        int count = (depth >= max_depth) ? (int)br_bits(tr, 8) : 1;
        for (int i = 0; i < count; i++) {
            int qd;
            if (br_bit(pr) == 0)
                qd = *prev + zigzag_dec(br_bits(pr, 8));
            else
                qd = (int)br_bits(pr, 16);
            *prev = qd;
            double d = dequantize((uint16_t)qd, max_d);
            int p = (*pos)++;
            out[p*3+0] = c.x * d;
            out[p*3+1] = c.y * d;
            out[p*3+2] = c.z * d;
        }
        return;
    }

    uint32_t mask = br_bits(tr, 4);
    tri subs[4];
    subdivide(v0, v1, v2, subs);

    for (int i = 0; i < 4; i++)
        if (mask & (1u << (3 - i)))
            dec_subtree(tr, pr, subs[i].v[0], subs[i].v[1], subs[i].v[2],
                        depth+1, max_depth, max_d, prev, out, pos);
}

/* -- Zlib helpers -- */

static uint8_t *zdeflate(const uint8_t *src, size_t slen, size_t *olen) {
    uLongf dlen = compressBound((uLong)slen);
    uint8_t *dst = malloc(dlen);
    if (!dst) return NULL;
    if (compress2(dst, &dlen, src, (uLong)slen, 9) != Z_OK) { free(dst); return NULL; }
    *olen = (size_t)dlen;
    return dst;
}

static uint8_t *zinflate(const uint8_t *src, size_t slen, size_t *olen) {
    size_t cap = slen * 4;
    if (cap < 4096) cap = 4096;
    uint8_t *buf = malloc(cap);
    for (;;) {
        uLongf dlen = (uLongf)cap;
        int ret = uncompress(buf, &dlen, src, (uLong)slen);
        if (ret == Z_OK)        { *olen = (size_t)dlen; return buf; }
        if (ret == Z_BUF_ERROR) { cap *= 2; buf = realloc(buf, cap); continue; }
        free(buf); *olen = 0; return NULL;
    }
}

/* ================================================================== */
/*  Shared preparation (spherical decomposition + face classification) */
/* ================================================================== */

typedef struct {
    v3       *dirs;
    uint16_t *qdist;
    int      *indices;      /* all indices, partitioned by face */
    int       fcounts[ICO_F];
    int       foff[ICO_F];
    icosahedron ico;
    uint8_t   max_depth;
    double    max_distance;
    double   *recon_out;
    uint32_t  n;
} prep_data;

static prep_data *prepare(const double *points, uint32_t n, gsc3_config cfg,
                          double *recon_out)
{
    prep_data *p = calloc(1, sizeof *p);
    if (!p) return NULL;

    if (cfg.max_distance < 1.0f) cfg.max_distance = 1.0f;
    if (cfg.max_distance > 65535.0f) cfg.max_distance = 65535.0f;
    cfg.max_distance = (float)(int)cfg.max_distance;

    p->n = n;
    p->max_depth = cfg.max_depth;
    p->max_distance = cfg.max_distance;
    p->recon_out = recon_out;

    p->dirs    = malloc(n * sizeof *p->dirs);
    p->qdist   = malloc(n * sizeof *p->qdist);
    p->indices = malloc(n * sizeof *p->indices);
    int *scratch = malloc(n * sizeof *scratch);
    if (!p->dirs || !p->qdist || !p->indices || !scratch) {
        free(p->dirs); free(p->qdist); free(p->indices);
        free(scratch); free(p);
        return NULL;
    }

    for (uint32_t i = 0; i < n; i++) {
        double x = points[i*3], y = points[i*3+1], z = points[i*3+2];
        double d = sqrt(x*x + y*y + z*z);
        double sd = d > 1e-15 ? d : 1e-15;
        p->dirs[i]  = (v3){x/sd, y/sd, z/sd};
        p->qdist[i] = quantize(d, p->max_distance);
    }

    ico_init(&p->ico);

    memset(p->fcounts, 0, sizeof p->fcounts);
    for (uint32_t i = 0; i < n; i++) {
        scratch[i] = classify_face(p->dirs[i], &p->ico);
        p->fcounts[scratch[i]]++;
    }

    p->foff[0] = 0;
    for (int fi = 1; fi < ICO_F; fi++)
        p->foff[fi] = p->foff[fi-1] + p->fcounts[fi-1];

    int fpos[ICO_F];
    memcpy(fpos, p->foff, sizeof p->foff);
    for (uint32_t i = 0; i < n; i++)
        p->indices[fpos[scratch[i]]++] = (int)i;

    if (recon_out) memset(recon_out, 0, (size_t)n * 3 * sizeof(double));

    free(scratch);
    return p;
}

static void prep_free(prep_data *p) {
    if (!p) return;
    free(p->dirs);
    free(p->qdist);
    free(p->indices);
    free(p);
}

/* ================================================================== */
/*  Encode a single face into a chunk                                  */
/* ================================================================== */

static int encode_face(const prep_data *p, int face_id,
                       const int *face_indices, int count,
                       double *recon_out, gsc3_chunk *out)
{
    int *idx = malloc(count * sizeof *idx);
    int *scratch = malloc(2 * count * sizeof *scratch);
    if (!idx || !scratch) { free(idx); free(scratch); return -1; }
    memcpy(idx, face_indices, count * sizeof *idx);

    bwriter tw = {0}, pw = {0};
    size_t init_cap = (size_t)count * 4;
    if (init_cap < 256) init_cap = 256;
    bw_init(&tw, init_cap);
    bw_init(&pw, init_cap);
    if (!tw.buf || !pw.buf) goto fail;

    v3 v0 = p->ico.verts[p->ico.faces[face_id][0]];
    v3 v1 = p->ico.verts[p->ico.faces[face_id][1]];
    v3 v2 = p->ico.verts[p->ico.faces[face_id][2]];

    int prev = 0;
    enc_subtree(&tw, &pw, v0, v1, v2,
                p->dirs, p->qdist, idx, count,
                recon_out, p->max_distance,
                0, p->max_depth, &prev, scratch);

    bw_flush(&tw);
    bw_flush(&pw);

    size_t tc_len, pc_len;
    uint8_t *tc = zdeflate(tw.buf, tw.len, &tc_len);
    uint8_t *pc = zdeflate(pw.buf, pw.len, &pc_len);
    if (!tc || !pc) { free(tc); free(pc); goto fail; }

    size_t total = 4 + tc_len + pc_len;
    uint8_t *buf = malloc(total);
    if (!buf) { free(tc); free(pc); goto fail; }

    buf[0] = (tc_len >> 24) & 0xFF;
    buf[1] = (tc_len >> 16) & 0xFF;
    buf[2] = (tc_len >> 8)  & 0xFF;
    buf[3] = tc_len & 0xFF;
    memcpy(buf + 4, tc, tc_len);
    memcpy(buf + 4 + tc_len, pc, pc_len);

    out->face_id     = (uint8_t)face_id;
    out->point_count = (uint32_t)count;
    out->data        = buf;
    out->data_len    = total;

    free(idx); free(scratch); free(tw.buf); free(pw.buf);
    free(tc); free(pc);
    return 0;

fail:
    free(idx); free(scratch); free(tw.buf); free(pw.buf);
    return -1;
}

/* ================================================================== */
/*  Parallel encode                                                    */
/* ================================================================== */

typedef struct {
    const prep_data *prep;
    int    face_id;
    const int *face_indices;
    int    count;
    double *recon_out;
    gsc3_chunk result;
    int    ok;
} enc_job;

static void *enc_worker(void *arg) {
    enc_job *j = arg;
    j->ok = encode_face(j->prep, j->face_id,
                        j->face_indices, j->count,
                        j->recon_out, &j->result);
    return NULL;
}

int gsc3_encode(const double *points, uint32_t n,
                gsc3_config cfg, gsc3_stream *out,
                double *recon_out)
{
    memset(out, 0, sizeof *out);
    if (n == 0) return -1;

    prep_data *p = prepare(points, n, cfg, recon_out);
    if (!p) return -1;

    int njobs = 0;
    enc_job jobs[ICO_F];
    pthread_t threads[ICO_F];

    for (int fi = 0; fi < ICO_F; fi++) {
        if (!p->fcounts[fi]) continue;
        enc_job *j = &jobs[njobs];
        j->prep = p;
        j->face_id = fi;
        j->face_indices = p->indices + p->foff[fi];
        j->count = p->fcounts[fi];
        j->recon_out = recon_out;
        j->ok = -1;
        njobs++;
    }

    for (int i = 0; i < njobs; i++)
        pthread_create(&threads[i], NULL, enc_worker, &jobs[i]);
    for (int i = 0; i < njobs; i++)
        pthread_join(threads[i], NULL);

    out->max_depth    = p->max_depth;
    out->max_distance = (uint16_t)p->max_distance;
    out->total_points = n;
    out->num_chunks   = njobs;

    for (int i = 0; i < njobs; i++) {
        if (jobs[i].ok != 0) { prep_free(p); return -1; }
        out->chunks[i] = jobs[i].result;
    }

    prep_free(p);
    return 0;
}

/* ================================================================== */
/*  Decode a single chunk                                              */
/* ================================================================== */

static double *decode_chunk_internal(uint8_t face_id,
                                     uint8_t max_depth, double max_dist,
                                     const uint8_t *data, size_t data_len,
                                     uint32_t expected_n)
{
    if (data_len < 4) return NULL;

    uint32_t tc_len = ((uint32_t)data[0]<<24) | ((uint32_t)data[1]<<16) |
                      ((uint32_t)data[2]<<8)  | data[3];
    if (4 + tc_len > data_len) return NULL;

    size_t traw_len, praw_len;
    uint8_t *traw = zinflate(data + 4, tc_len, &traw_len);
    uint8_t *praw = zinflate(data + 4 + tc_len, data_len - 4 - tc_len, &praw_len);
    if (!traw || !praw) { free(traw); free(praw); return NULL; }

    double *out = malloc((size_t)expected_n * 3 * sizeof(double));
    if (!out) { free(traw); free(praw); return NULL; }

    breader tr, pr;
    br_init(&tr, traw, traw_len);
    br_init(&pr, praw, praw_len);

    icosahedron ico;
    ico_init(&ico);

    v3 v0 = ico.verts[ico.faces[face_id][0]];
    v3 v1 = ico.verts[ico.faces[face_id][1]];
    v3 v2 = ico.verts[ico.faces[face_id][2]];

    int prev = 0, pos = 0;
    dec_subtree(&tr, &pr, v0, v1, v2,
                0, max_depth, max_dist, &prev, out, &pos);

    free(traw);
    free(praw);
    return out;
}

double *gsc3_decode_chunk(const gsc3_chunk *chunk,
                          uint8_t max_depth, uint16_t max_distance,
                          uint32_t *out_n)
{
    *out_n = chunk->point_count;
    return decode_chunk_internal(chunk->face_id, max_depth,
                                (double)max_distance,
                                chunk->data, chunk->data_len,
                                chunk->point_count);
}

/* ================================================================== */
/*  Parallel decode                                                    */
/* ================================================================== */

typedef struct {
    uint8_t  face_id;
    uint8_t  max_depth;
    double   max_dist;
    const uint8_t *data;
    size_t   data_len;
    uint32_t point_count;
    double  *out;       /* pre-allocated slice of the combined array */
} dec_job;

static void *dec_worker(void *arg) {
    dec_job *j = arg;
    if (j->data_len < 4) return NULL;

    uint32_t tc_len = ((uint32_t)j->data[0]<<24) | ((uint32_t)j->data[1]<<16) |
                      ((uint32_t)j->data[2]<<8)  | j->data[3];

    size_t traw_len, praw_len;
    uint8_t *traw = zinflate(j->data + 4, tc_len, &traw_len);
    uint8_t *praw = zinflate(j->data + 4 + tc_len,
                             j->data_len - 4 - tc_len, &praw_len);
    if (!traw || !praw) { free(traw); free(praw); return NULL; }

    breader tr, pr;
    br_init(&tr, traw, traw_len);
    br_init(&pr, praw, praw_len);

    icosahedron ico;
    ico_init(&ico);

    v3 v0 = ico.verts[ico.faces[j->face_id][0]];
    v3 v1 = ico.verts[ico.faces[j->face_id][1]];
    v3 v2 = ico.verts[ico.faces[j->face_id][2]];

    int prev = 0, pos = 0;
    dec_subtree(&tr, &pr, v0, v1, v2,
                0, j->max_depth, j->max_dist, &prev, j->out, &pos);

    free(traw);
    free(praw);
    return NULL;
}

double *gsc3_decode(const gsc3_stream *stream, uint32_t *out_n) {
    uint32_t total = stream->total_points;
    double *out = malloc((size_t)total * 3 * sizeof(double));
    if (!out) return NULL;

    int nj = stream->num_chunks;
    dec_job jobs[GSC3_MAX_FACES];
    pthread_t threads[GSC3_MAX_FACES];

    uint32_t offset = 0;
    for (int i = 0; i < nj; i++) {
        const gsc3_chunk *c = &stream->chunks[i];
        dec_job *j = &jobs[i];
        j->face_id     = c->face_id;
        j->max_depth   = stream->max_depth;
        j->max_dist    = (double)stream->max_distance;
        j->data        = c->data;
        j->data_len    = c->data_len;
        j->point_count = c->point_count;
        j->out         = out + (size_t)offset * 3;
        offset += c->point_count;
    }

    for (int i = 0; i < nj; i++)
        pthread_create(&threads[i], NULL, dec_worker, &jobs[i]);
    for (int i = 0; i < nj; i++)
        pthread_join(threads[i], NULL);

    *out_n = total;
    return out;
}

/* ================================================================== */
/*  Streaming encoder context                                          */
/* ================================================================== */

struct gsc3_enc_ctx {
    prep_data *prep;
    int        next_face;   /* 0..ICO_F, scans for non-empty */
};

gsc3_enc_ctx *gsc3_enc_begin(const double *points, uint32_t n,
                             gsc3_config cfg, double *recon_out)
{
    gsc3_enc_ctx *ctx = calloc(1, sizeof *ctx);
    if (!ctx) return NULL;
    ctx->prep = prepare(points, n, cfg, recon_out);
    if (!ctx->prep) { free(ctx); return NULL; }
    ctx->next_face = 0;
    return ctx;
}

int gsc3_enc_next(gsc3_enc_ctx *ctx, gsc3_chunk *out) {
    prep_data *p = ctx->prep;
    while (ctx->next_face < ICO_F && p->fcounts[ctx->next_face] == 0)
        ctx->next_face++;
    if (ctx->next_face >= ICO_F) return -1;

    int fi = ctx->next_face++;
    return encode_face(p, fi,
                       p->indices + p->foff[fi], p->fcounts[fi],
                       p->recon_out, out);
}

void gsc3_enc_free(gsc3_enc_ctx *ctx) {
    if (!ctx) return;
    prep_free(ctx->prep);
    free(ctx);
}

/* ================================================================== */
/*  Serialization                                                      */
/* ================================================================== */

uint8_t *gsc3_serialize(const gsc3_stream *s, size_t *out_len) {
    size_t total = 16;
    for (int i = 0; i < s->num_chunks; i++)
        total += 8 + s->chunks[i].data_len;

    uint8_t *buf = malloc(total);
    if (!buf) return NULL;

    memcpy(buf, "GSC3", 4);
    buf[4] = s->max_depth;
    buf[5] = 0x07;  /* delta + split + streaming */
    buf[6] = s->max_distance >> 8;
    buf[7] = s->max_distance & 0xFF;
    buf[8]  = (s->total_points >> 24) & 0xFF;
    buf[9]  = (s->total_points >> 16) & 0xFF;
    buf[10] = (s->total_points >> 8)  & 0xFF;
    buf[11] = s->total_points & 0xFF;
    buf[12] = (uint8_t)s->num_chunks;
    buf[13] = buf[14] = buf[15] = 0;

    size_t off = 16;
    for (int i = 0; i < s->num_chunks; i++) {
        const gsc3_chunk *c = &s->chunks[i];
        buf[off++] = c->face_id;
        buf[off++] = (c->point_count >> 16) & 0xFF;
        buf[off++] = (c->point_count >> 8)  & 0xFF;
        buf[off++] = c->point_count & 0xFF;
        uint32_t dl = (uint32_t)c->data_len;
        buf[off++] = (dl >> 24) & 0xFF;
        buf[off++] = (dl >> 16) & 0xFF;
        buf[off++] = (dl >> 8)  & 0xFF;
        buf[off++] = dl & 0xFF;
        memcpy(buf + off, c->data, c->data_len);
        off += c->data_len;
    }

    *out_len = total;
    return buf;
}

int gsc3_deserialize(const uint8_t *data, size_t len, gsc3_stream *out) {
    if (len < 16 || memcmp(data, "GSC3", 4) != 0) return -1;

    memset(out, 0, sizeof *out);
    out->max_depth    = data[4];
    out->max_distance = (uint16_t)((data[6] << 8) | data[7]);
    out->total_points = ((uint32_t)data[8]<<24) | ((uint32_t)data[9]<<16) |
                        ((uint32_t)data[10]<<8) | data[11];
    out->num_chunks   = data[12];

    size_t off = 16;
    for (int i = 0; i < out->num_chunks; i++) {
        if (off + 8 > len) return -1;
        gsc3_chunk *c = &out->chunks[i];
        c->face_id     = data[off];
        c->point_count = ((uint32_t)data[off+1]<<16) |
                         ((uint32_t)data[off+2]<<8)  | data[off+3];
        uint32_t dl    = ((uint32_t)data[off+4]<<24) |
                         ((uint32_t)data[off+5]<<16) |
                         ((uint32_t)data[off+6]<<8)  | data[off+7];
        off += 8;
        if (off + dl > len) return -1;
        c->data = malloc(dl);
        if (!c->data) return -1;
        memcpy(c->data, data + off, dl);
        c->data_len = dl;
        off += dl;
    }

    return 0;
}

/* ================================================================== */
/*  Cleanup                                                            */
/* ================================================================== */

void gsc3_stream_free(gsc3_stream *s) {
    for (int i = 0; i < s->num_chunks; i++)
        free(s->chunks[i].data);
    memset(s, 0, sizeof *s);
}
