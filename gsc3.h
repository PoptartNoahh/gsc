/*
 * GSC v3 - Geodesic Subdivision Compression (streaming + parallel)
 *
 * Same core algorithm as GSC v2 but each icosahedron face is an
 * independent chunk that can be encoded/decoded in parallel or
 * streamed one at a time.  Up to 20-way parallelism.
 *
 * Requires: zlib (-lz), pthreads (-lpthread)
 */

#ifndef GSC3_H
#define GSC3_H

#include <stdint.h>
#include <stddef.h>

#define GSC3_MAX_FACES 20

typedef struct {
    uint8_t  max_depth;      /* subdivision depth cap (default 8)   */
    float    max_distance;   /* distance range ceiling in metres    */
    int      threads;        /* 0 = one thread per non-empty face   */
} gsc3_config;

/* One independently encoded face chunk */
typedef struct {
    uint8_t  face_id;
    uint32_t point_count;
    uint8_t *data;           /* topo_len(4) + topo_z + pay_z        */
    size_t   data_len;
} gsc3_chunk;

/* Complete stream = header metadata + array of chunks */
typedef struct {
    uint8_t    max_depth;
    uint16_t   max_distance;
    uint32_t   total_points;
    int        num_chunks;
    gsc3_chunk chunks[GSC3_MAX_FACES];
} gsc3_stream;

/*
 * Parallel encode.  Classifies points into icosahedron faces, then
 * encodes each face as an independent chunk using one thread per face.
 * If recon_out is non-NULL, fills it with n*3 reconstructed doubles.
 * Returns 0 on success.  Caller must call gsc3_stream_free().
 */
int gsc3_encode(const double *points, uint32_t n,
                gsc3_config cfg, gsc3_stream *out,
                double *recon_out);

/*
 * Parallel decode.  Decodes all chunks simultaneously.
 * Returns malloc'd array of out_n*3 doubles.  Caller frees.
 */
double *gsc3_decode(const gsc3_stream *stream, uint32_t *out_n);

/*
 * Decode a single chunk.  For true streaming - decode as chunks arrive.
 * Returns malloc'd array of out_n*3 doubles.  Caller frees.
 */
double *gsc3_decode_chunk(const gsc3_chunk *chunk,
                          uint8_t max_depth, uint16_t max_distance,
                          uint32_t *out_n);

/* --- Streaming encoder (one face at a time) --- */

typedef struct gsc3_enc_ctx gsc3_enc_ctx;

/*
 * Begin streaming encode.  Prepares spherical decomposition and face
 * classification.  Call gsc3_enc_next() repeatedly to get one chunk
 * at a time.  recon_out may be NULL.
 */
gsc3_enc_ctx *gsc3_enc_begin(const double *points, uint32_t n,
                             gsc3_config cfg, double *recon_out);

/*
 * Encode the next non-empty face.  Returns 0 on success, -1 when done.
 * Chunk data is malloc'd - caller must free chunk->data.
 */
int gsc3_enc_next(gsc3_enc_ctx *ctx, gsc3_chunk *out);

void gsc3_enc_free(gsc3_enc_ctx *ctx);

/* --- Serialization --- */

uint8_t *gsc3_serialize(const gsc3_stream *stream, size_t *out_len);
int      gsc3_deserialize(const uint8_t *data, size_t len, gsc3_stream *out);

void gsc3_stream_free(gsc3_stream *s);

#endif /* GSC3_H */
