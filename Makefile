CC = gcc
CFLAGS = -O2 -Wall -Wextra
INC = -Iexternal/mavlink

all:
	$(CC) $(CFLAGS) main.c -o ground_rover_daemon $(INC)

clean:
	rm -f ground_rover_daemon