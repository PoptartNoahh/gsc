/*
 * GSC v2 - C reference implementation.
 *
 * Sections:
 *   1. Vector math
 *   2. Icosahedron geometry
 *   3. Bit-level I/O
 *   4. Point classification
 *   5. Encoder
 *   6. Decoder
 */

#include "gsc.h"

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define GSC_MAGIC       "GSC2"
#define GSC_HEADER      16
#define FLAG_DELTA      0x01
#define FLAG_SPLIT      0x02

/* ------------------------------------------------------------------ */
/*  1. Vector math                                                     */
/* ------------------------------------------------------------------ */

typedef struct { double x, y, z; } v3;

static inline v3  v3_add(v3 a, v3 b)     { return (v3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline v3  v3_sub(v3 a, v3 b)     { return (v3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline v3  v3_scale(v3 a, double s){ return (v3){a.x*s, a.y*s, a.z*s}; }
static inline double v3_dot(v3 a, v3 b)  { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline v3  v3_cross(v3 a, v3 b) {
    return (v3){ a.y*b.z - a.z*b.y,
                 a.z*b.x - a.x*b.z,
                 a.x*b.y - a.y*b.x };
}
static inline double v3_len(v3 a) { return sqrt(v3_dot(a, a)); }
static inline v3 v3_norm(v3 a) {
    double n = v3_len(a);
    return n > 1e-15 ? v3_scale(a, 1.0/n) : a;
}
static inline v3 v3_centroid(v3 a, v3 b, v3 c) {
    return v3_norm(v3_add(v3_add(a, b), c));
}

/* ------------------------------------------------------------------ */
/*  2. Icosahedron                                                     */
/* ------------------------------------------------------------------ */

#define ICO_V 12
#define ICO_F 20

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

    /* fix winding so cross(v1-v0, v2-v0) points outward */
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

/* ------------------------------------------------------------------ */
/*  3. Bit-level I/O                                                   */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t *buf;
    size_t   cap, len;
    uint8_t  cur;
    int      pos;       /* bits written into cur, 0-7 */
} bwriter;

static void bw_init(bwriter *w, size_t cap) {
    w->buf = malloc(cap);
    w->cap = cap;
    w->len = 0;
    w->cur = 0;
    w->pos = 0;
}

static void bw_bit(bwriter *w, int bit) {
    w->cur = (w->cur << 1) | (bit & 1);
    if (++w->pos == 8) {
        if (w->len >= w->cap) { w->cap *= 2; w->buf = realloc(w->buf, w->cap); }
        w->buf[w->len++] = w->cur;
        w->cur = 0;
        w->pos = 0;
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
        w->cur = 0;
        w->pos = 0;
    }
}

typedef struct {
    const uint8_t *data;
    size_t len;
    size_t byte;
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

/* ------------------------------------------------------------------ */
/*  4. Point classification                                            */
/* ------------------------------------------------------------------ */

static int point_in_tri(v3 dir, v3 v0, v3 v1, v3 v2) {
    double s0 = v3_dot(dir, v3_cross(v0, v1));
    double s1 = v3_dot(dir, v3_cross(v1, v2));
    double s2 = v3_dot(dir, v3_cross(v2, v0));
    return s0 >= 0 && s1 >= 0 && s2 >= 0;
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

/* ------------------------------------------------------------------ */
/*  5. Encoder                                                         */
/* ------------------------------------------------------------------ */

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

/*
 * Encode one subtree.  idx[0..n-1] is modified in-place during
 * partitioning.  scratch is a pre-allocated work buffer (2*N ints)
 * shared across all recursive calls -- zero allocs in the hot path.
 */
static void enc_subtree(bwriter *tw, bwriter *pw,
                        v3 v0, v3 v1, v3 v2,
                        const v3 *dirs, const uint16_t *qdist,
                        int *idx, int n,
                        double *recon, double max_d,
                        int depth, int max_depth, int *prev,
                        int *scratch)
{
    assert(n > 0);

    /* leaf when: single point, or at max_depth with count <= 255.
       if count > 255 at max_depth, keep subdividing so the 8-bit
       count field doesn't overflow.  hard cap at max_depth+8 to
       prevent runaway recursion on degenerate input. */
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

    /* classify each point into one of 4 sub-triangles */
    int counts[4] = {0};
    for (int i = 0; i < n; i++) {
        scratch[i] = classify_sub(dirs[idx[i]], subs);
        counts[scratch[i]]++;
    }

    uint32_t mask = 0;
    for (int c = 0; c < 4; c++)
        if (counts[c]) mask |= 1u << (3 - c);
    bw_bits(tw, mask, 4);

    /* stable partition idx in-place via scratch as temp */
    int off[4], pos[4];
    off[0] = 0;
    for (int c = 1; c < 4; c++) off[c] = off[c-1] + counts[c-1];
    memcpy(pos, off, sizeof(off));

    int *tmp = scratch + n;
    for (int i = 0; i < n; i++)
        tmp[pos[scratch[i]]++] = idx[i];
    memcpy(idx, tmp, n * sizeof(int));

    /* recurse into non-empty children */
    for (int c = 0; c < 4; c++) {
        if (counts[c])
            enc_subtree(tw, pw, subs[c].v[0], subs[c].v[1], subs[c].v[2],
                        dirs, qdist, idx + off[c], counts[c],
                        recon, max_d, depth+1, max_depth, prev,
                        scratch);
    }
}

static uint8_t *zdeflate(const uint8_t *src, size_t slen, size_t *olen) {
    uLongf dlen = compressBound((uLong)slen);
    uint8_t *dst = malloc(dlen);
    if (!dst) return NULL;
    if (compress2(dst, &dlen, src, (uLong)slen, 9) != Z_OK) { free(dst); return NULL; }
    *olen = (size_t)dlen;
    return dst;
}

uint8_t *gsc_encode(const double *points, uint32_t n,
                    gsc_config cfg, size_t *out_size,
                    double *recon_out)
{
    v3       *dirs    = NULL;
    uint16_t *qdist   = NULL;
    int      *indices = NULL;
    int      *scratch = NULL;
    uint8_t  *tc = NULL, *pc = NULL, *out = NULL;
    bwriter   tw = {0}, pw = {0};
    uint32_t  fmask = 0;

    if (n == 0) { *out_size = 0; return NULL; }

    /* max_distance is stored as uint16 in the header -- clamp it */
    if (cfg.max_distance < 1.0f) cfg.max_distance = 1.0f;
    if (cfg.max_distance > 65535.0f) cfg.max_distance = 65535.0f;
    cfg.max_distance = (float)(int)cfg.max_distance;

    dirs    = malloc(n * sizeof *dirs);
    qdist   = malloc(n * sizeof *qdist);
    indices = malloc(n * sizeof *indices);
    scratch = malloc(2 * n * sizeof *scratch);
    if (!dirs || !qdist || !indices || !scratch)
        goto fail;

    /* spherical decomposition: direction + quantised distance */
    for (uint32_t i = 0; i < n; i++) {
        double x = points[i*3], y = points[i*3+1], z = points[i*3+2];
        double d = sqrt(x*x + y*y + z*z);
        double sd = d > 1e-15 ? d : 1e-15;
        dirs[i]  = (v3){x/sd, y/sd, z/sd};
        qdist[i] = quantize(d, cfg.max_distance);
    }

    icosahedron ico;
    ico_init(&ico);

    /* classify points into faces, then stable-partition indices by face */
    int fcounts[ICO_F] = {0};
    for (uint32_t i = 0; i < n; i++) {
        scratch[i] = classify_face(dirs[i], &ico);
        fcounts[scratch[i]]++;
    }

    int foff[ICO_F], fpos[ICO_F];
    foff[0] = 0;
    for (int fi = 1; fi < ICO_F; fi++) foff[fi] = foff[fi-1] + fcounts[fi-1];
    memcpy(fpos, foff, sizeof(foff));

    for (uint32_t i = 0; i < n; i++)
        indices[fpos[scratch[i]]++] = (int)i;

    for (int fi = 0; fi < ICO_F; fi++)
        if (fcounts[fi]) fmask |= 1u << (19 - fi);

    /* encode tree into split streams */
    bw_init(&tw, 1 << 16);
    bw_init(&pw, 1 << 16);
    if (!tw.buf || !pw.buf) goto fail;

    if (recon_out) memset(recon_out, 0, (size_t)n * 3 * sizeof(double));

    for (int fi = 0; fi < ICO_F; fi++) {
        if (!fcounts[fi]) continue;
        v3 v0 = ico.verts[ico.faces[fi][0]];
        v3 v1 = ico.verts[ico.faces[fi][1]];
        v3 v2 = ico.verts[ico.faces[fi][2]];
        int prev = 0;
        enc_subtree(&tw, &pw, v0, v1, v2,
                    dirs, qdist, indices + foff[fi], fcounts[fi],
                    recon_out, cfg.max_distance,
                    0, cfg.max_depth, &prev,
                    scratch);
    }

    bw_flush(&tw);
    bw_flush(&pw);

    /* compress each stream independently */
    size_t tc_len, pc_len;
    tc = zdeflate(tw.buf, tw.len, &tc_len);
    pc = zdeflate(pw.buf, pw.len, &pc_len);
    if (!tc || !pc) goto fail;

    /* assemble: header + topo_len + topo + payload */
    size_t total = GSC_HEADER + 4 + tc_len + pc_len;
    out = malloc(total);
    if (!out) goto fail;

    memcpy(out, GSC_MAGIC, 4);
    out[4]  = cfg.max_depth;
    out[5]  = FLAG_DELTA | FLAG_SPLIT;
    uint16_t md = (uint16_t)cfg.max_distance;
    out[6]  = md >> 8;   out[7]  = md & 0xFF;
    out[8]  = (n >> 24);  out[9]  = (n >> 16) & 0xFF;
    out[10] = (n >> 8) & 0xFF;  out[11] = n & 0xFF;
    out[12] = (fmask >> 16) & 0xFF;
    out[13] = (fmask >> 8)  & 0xFF;
    out[14] = fmask & 0xFF;
    out[15] = 0;

    out[16] = (tc_len >> 24) & 0xFF;
    out[17] = (tc_len >> 16) & 0xFF;
    out[18] = (tc_len >> 8)  & 0xFF;
    out[19] = tc_len & 0xFF;
    memcpy(out + 20, tc, tc_len);
    memcpy(out + 20 + tc_len, pc, pc_len);

    *out_size = total;
    goto done;

fail:
    free(out);
    out = NULL;
    *out_size = 0;

done:
    free(dirs);
    free(qdist);
    free(indices);
    free(scratch);
    free(tw.buf);
    free(pw.buf);
    free(tc);
    free(pc);
    return out;
}

/* ------------------------------------------------------------------ */
/*  6. Decoder                                                         */
/* ------------------------------------------------------------------ */

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

double *gsc_decode(const uint8_t *data, size_t len, uint32_t *out_n) {
    if (len < GSC_HEADER || memcmp(data, GSC_MAGIC, 4) != 0) return NULL;

    int max_depth    = data[4];
    double max_dist  = (double)((data[6] << 8) | data[7]);
    uint32_t npts    = ((uint32_t)data[8]<<24) | ((uint32_t)data[9]<<16) |
                       ((uint32_t)data[10]<<8) | data[11];
    uint32_t fmask   = ((uint32_t)data[12]<<16) | ((uint32_t)data[13]<<8) | data[14];

    const uint8_t *body = data + GSC_HEADER;
    uint32_t tc_len = ((uint32_t)body[0]<<24) | ((uint32_t)body[1]<<16) |
                      ((uint32_t)body[2]<<8)  | body[3];

    size_t traw_len, praw_len;
    uint8_t *traw = zinflate(body + 4, tc_len, &traw_len);
    uint8_t *praw = zinflate(body + 4 + tc_len, len - GSC_HEADER - 4 - tc_len, &praw_len);
    if (!traw || !praw) { free(traw); free(praw); return NULL; }

    breader tr, pr;
    br_init(&tr, traw, traw_len);
    br_init(&pr, praw, praw_len);

    icosahedron ico;
    ico_init(&ico);

    double *out = malloc((size_t)npts * 3 * sizeof(double));
    int pos = 0;

    for (int fi = 0; fi < ICO_F; fi++) {
        if (!(fmask & (1u << (19 - fi)))) continue;
        v3 v0 = ico.verts[ico.faces[fi][0]];
        v3 v1 = ico.verts[ico.faces[fi][1]];
        v3 v2 = ico.verts[ico.faces[fi][2]];
        int prev = 0;
        dec_subtree(&tr, &pr, v0, v1, v2,
                    0, max_depth, max_dist, &prev, out, &pos);
    }

    free(traw);
    free(praw);

    assert(pos == (int)npts);
    *out_n = npts;
    return out;
}
