# crxdec: Canon CRX frame decoder used by crx2mlv.py (LibRaw's own CRX code, LibRaw 0.22.1).
# Get LibRaw 0.22.1 (https://www.libraw.org, LGPL-2.1 / CDDL), build it (make -f Makefile.dist), then:
#     make LIBRAW=/path/to/LibRaw
LIBRAW ?= ../../../../LibRaw
CXX ?= g++

crxdec: crxdec.cpp
	$(CXX) -O2 -DLIBRAW_LIBRARY_BUILD -DLIBRAW_NOTHREADS -DUSE_ZLIB -I$(LIBRAW) -o $@ $< $(LIBRAW)/lib/libraw.a -lz -lm

clean:
	rm -f crxdec
.PHONY: clean
