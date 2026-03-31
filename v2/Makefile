CC      = cc
CFLAGS  = -O2 -Wall -Wextra -std=c99
LDFLAGS = -lz -lm

gsc: main.o gsc.o
	$(CC) -o $@ $^ $(LDFLAGS)

main.o: main.c gsc.h
	$(CC) $(CFLAGS) -c main.c

gsc.o: gsc.c gsc.h
	$(CC) $(CFLAGS) -c gsc.c

clean:
	rm -f *.o gsc

.PHONY: clean
