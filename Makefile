CC      = cc
CFLAGS  = -O2 -Wall -Wextra -std=c99
LDFLAGS = -lz -lm -lpthread

stream_bench: stream_bench.o gsc3.o
	$(CC) -o $@ $^ $(LDFLAGS)

stream_bench.o: stream_bench.c gsc3.h
	$(CC) $(CFLAGS) -c stream_bench.c

gsc3.o: gsc3.c gsc3.h
	$(CC) $(CFLAGS) -c gsc3.c

clean:
	rm -f *.o stream_bench

.PHONY: clean
