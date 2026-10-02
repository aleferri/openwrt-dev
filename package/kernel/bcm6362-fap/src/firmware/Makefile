# SPDX-License-Identifier: GPL-2.0-or-later

CROSS_COMPILE ?= mips-linux-gnu-

CC := $(CROSS_COMPILE)gcc
OBJCOPY := $(CROSS_COMPILE)objcopy

CFLAGS := -march=mips32r2 -mtune=4kec -EB -msoft-float -mno-abicalls -fno-pic -G0 \
	  -Os -ffreestanding -fno-builtin -fno-asynchronous-unwind-tables \
	  -Wall -Wextra -I..
LDFLAGS := -nostdlib -static -Wl,--build-id=none

OBJS := start.o bench.o main.o

all: bcm6362-fap.bin

bcm6362-fap.bin: bcm6362-fap.elf
	$(OBJCOPY) -O binary $< $@

bcm6362-fap.elf: $(OBJS) fap.lds
	$(CC) $(CFLAGS) $(LDFLAGS) -T fap.lds -o $@ $(OBJS)

fap.lds: fap.lds.S ../fap_shared.h fap4ke.h
	$(CC) -E -P -undef -x c -D__ASSEMBLER__ -I.. -o $@ $<

%.o: %.S ../fap_shared.h fap4ke.h
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.c ../fap_shared.h fap4ke.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) fap.lds bcm6362-fap.elf bcm6362-fap.bin

.PHONY: all clean
