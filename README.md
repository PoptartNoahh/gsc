# gsc - geodesic subdivision compression
Lossy point cloud compression using geodesic subdivision. Encodes angular positions as paths through an icosahedral tree instead of storing floats.

~6.2:1 ratio on real lidar data. Parallel encode/decode via pthreads (one thread per icosahedron face, up to 20-way). Chunks are independent - stream and decode them as they arrive.

## numbers
- 6.2:1 compression (1,418 KB -> 227 KB per frame)
- parallel encode: 20M pts/s
- parallel decode: 163M pts/s (full 121K-point frame in 0.7ms)
- first decoded points in 0.3ms (don't wait for full frame)
- error below sensor noise (mean 0.09 deg, 1.5mm)

## real-world
over wifi (50 Mbps): raw takes 227ms/frame, gsc takes 38ms. 4 fps becomes 26 fps.
over LTE (20 Mbps): raw 567ms, gsc 92ms. real-time streaming where it wasn't possible.
over gigabit: raw 11ms, gsc 2.8ms. still 4x faster.

## build
```
make
```

## benchmark
```
./stream_bench input.bin [depth] [max_dist]
```

Input .bin is flat float32: `[x, y, z, intensity]` per point (KITTI velodyne format). Intensity is ignored.

## as a library
```c
#include "gsc3.h"

gsc3_config cfg = { .max_depth = 8, .max_distance = 200.0f, .threads = 0 };

// parallel encode (all faces at once)
gsc3_stream stream;
gsc3_encode(points, n, cfg, &stream, NULL);

// parallel decode
uint32_t out_n;
double *pts = gsc3_decode(&stream, &out_n);

// or stream chunk-by-chunk
gsc3_enc_ctx *enc = gsc3_enc_begin(points, n, cfg, NULL);
gsc3_chunk chunk;
while (gsc3_enc_next(enc, &chunk) == 0) {
    uint32_t cn;
    double *cp = gsc3_decode_chunk(&chunk, cfg.max_depth, 200, &cn);
    // use cp immediately
    free(cp);
    free(chunk.data);
}
gsc3_enc_free(enc);
```
