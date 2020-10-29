ifeq ($(OS),Windows_NT)
else
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S), Linux)
		OS := Linux
	endif
	ifeq ($(UNAME_S), Darwin)
		OS := OSX
	endif
endif

ifeq ($(OS), Linux)
VASM := vasm/vasm6502_oldstyle
endif

ifeq ($(OS), OSX)
VASM := vasm/vasm6502_oldstyle_osx
endif


all: main test/rom.bin

main: main.c emu6502.h emu6502_opcodes.h
	clang main.c -o main

test/rom.bin: test/rom.s
	$(VASM) -Fbin -dotdir test/rom.s -o test/rom.bin


