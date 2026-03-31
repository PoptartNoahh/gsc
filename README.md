# gsc - geodesic subdivision compression
Lossy point cloud compression using geodesic subdivision. Encodes angular positions as paths through an icosahedral tree instead of storing floats.

~6.5:1 ratio on real lidar data, sub-millisecond decode. Single file, C99, only needs zlib.

## build
```
make
```

## cli
```
gsc encode input.bin output.gsc
gsc decode output.gsc output.bin
gsc bench  input.bin
```

Input .bin is flat float32: `[x, y, z, intensity]` per point (KITTI velodyne format). Intensity is ignored. Decoded .bin uses the same layout with intensity zeroed.

## as a library
```c
#include "gsc.h"

gsc_config cfg = { .max_depth = 8, .max_distance = 200.0f };
size_t size;
uint8_t *buf = gsc_encode(points, n, cfg, &size, NULL);

uint32_t out_n;
double *pts = gsc_decode(buf, size, &out_n);
```
