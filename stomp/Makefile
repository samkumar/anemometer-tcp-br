CC = gcc
CFLAGS = -O3 -Wall -Wpedantic
LDFLAGS = -lrt

all: stomp

stomp: stomp.o
	$(CC) $(CFLAGS) stomp.o $(LDFLAGS) -o stomp

stomp.o: stomp.c
	$(CC) $(CFLAGS) -c stomp.c -o stomp.o

clean:
	rm -f stomp stomp.o
