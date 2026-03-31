/*
 * GSC v2 - Geodesic Subdivision Compression
 *
 * Encodes 3D point clouds by representing angular positions as paths
 * through an icosahedral subdivision tree.  Distances are delta-coded
 * and topology/payload streams are zlib-compressed independently.
 *
 * Requires: zlib (-lz)
 */

#ifndef GSC_H
#define GSC_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint8_t  max_depth;     /* subdivision depth cap (default 8) */
    float    max_distance;  /* distance range ceiling in metres  */
} gsc_config;

/*
 * Encode `n` points into GSC2 format.
 *
 *   points    - flat array of n*3 doubles (x,y,z interleaved)
 *   cfg       - encoding parameters
 *   out_size  - receives the byte length of the returned buffer
 *   recon_out - if non-NULL, filled with n*3 reconstructed doubles
 *               (same index order as input, for error measurement)
 *
 * Returns a malloc'd buffer.  Caller must free().
 * Returns NULL on failure.
 */
uint8_t *gsc_encode(const double *points, uint32_t n,
                    gsc_config cfg,
                    size_t *out_size,
                    double *recon_out);

/*
 * Decode a GSC2 buffer into a point cloud.
 *
 *   data   - GSC2 byte buffer
 *   len    - byte length of data
 *   out_n  - receives the point count
 *
 * Returns a malloc'd array of out_n×3 doubles.  Caller must free().
 * Returns NULL on failure.
 */
double *gsc_decode(const uint8_t *data, size_t len, uint32_t *out_n);

#endif /* GSC_H */
