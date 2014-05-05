CC=gcc
CFLAGS=-Wall -Wextra
LDFLAGS=-lm

.PHONY: all clean mrproper

all: test_position_manager

test_position_manager: test_position_manager.o position_manager.o

position_manager.o: ../utils/position_manager.c ../utils/position_manager.h
	${CC} ${CFLAGS} -o $@ -c $<

%.o: %.c
	${CC} ${CFLAGS} -c $<

clean:
	@ rm -f *.o *~
