# musrender - render Doom MUS lumps to WAV with Chocolate Doom's OPL code.
CC      ?= gcc
CFLAGS  ?= -O2 -g
CFLAGS  += -std=gnu11 -Wall -Wno-unused-function -Wno-unused-variable \
           -Wno-unused-but-set-variable -Wno-format-truncation
# stubs/ first so its config.h / SDL.h / SDL_endian.h win.
CPPFLAGS += -Istubs -Ichoc -Isrc
LDLIBS  += -lm

WAD     ?= DOOM1.WAD
OUTDIR  ?= music

CHOC_SRC = choc/opl.c choc/opl_queue.c choc/opl3.c \
           choc/i_oplmusic.c choc/mus2mid.c choc/midifile.c choc/memio.c
OWN_SRC  = src/opl_offline.c src/engine_stubs.c src/wadfile.c src/musrender.c
OBJ      = $(patsubst %.c,build/%.o,$(CHOC_SRC) $(OWN_SRC))

all: musrender

musrender: $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

build/%.o: %.c $(wildcard stubs/*.h choc/*.h src/*.h)
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

render: musrender
	./musrender -o $(OUTDIR) $(WAD)

clean:
	rm -rf build musrender

.PHONY: all clean render
