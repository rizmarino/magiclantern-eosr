// crxdec: decode one Canon CRX (CR3) compressed raw image from a CMP1 header + compressed sample, using LibRaw's
// own CRX decoder (LibRaw 0.22.1 internals via a subclass). Output: raw 16-bit little-endian samples, f_width x
// f_height, CFA order as in the stream.
// Usage: crxdec <cmp1.bin> <sample.bin> <out.u16> -> prints "W H bits planes encType levels"
// Purpose: step 2 of the EOS R lossless 4K work (check the camera's encoder output is bit-exact) and the core
// of a later MLV(CRX) -> DNG converter. Validate first on real CR3 samples (crx_extract.py).
#include "libraw/libraw.h"
#include <cstdio>
#include <cstdlib>
#include <vector>

class CrxH : public LibRaw
{
public:
    int decode(std::vector<unsigned char> &cmp1, std::vector<char> &blob, std::vector<unsigned short> &out,
               int &w, int &h)
    {
        if (crxParseImageHeader(cmp1.data(), 0, cmp1.size())) return -1;
        crx_data_header_t &hd = libraw_internal_data.unpacker_data.crx_header[0];
        w = hd.f_width; h = hd.f_height;
        printf("%d %d %d %d %d %d mdatHdr %d\n", hd.f_width, hd.f_height, hd.nBits, hd.nPlanes, hd.encType,
               hd.imageLevels, hd.mdatHdrSize);
        out.assign((size_t)w * h, 0);
        libraw_internal_data.unpacker_data.crx_track_selected = 0;
        LibRaw_buffer_datastream *ds = new LibRaw_buffer_datastream(blob.data(), blob.size());
        libraw_internal_data.internal_data.input = ds;
        libraw_internal_data.unpacker_data.data_offset = 0;
        libraw_internal_data.unpacker_data.data_size = blob.size();
        imgdata.sizes.raw_width = w; imgdata.sizes.raw_height = h; imgdata.sizes.raw_pitch = w * 2;
        imgdata.rawdata.raw_image = out.data();
        int rc = 0;
        try { crxLoadRaw(); }
        catch (...) { rc = -2; }
        imgdata.rawdata.raw_image = 0;
        libraw_internal_data.internal_data.input = 0;
        delete ds;
        return rc;
    }
};

template <class T> static bool slurp(const char *p, std::vector<T> &v)
{
    FILE *f = fopen(p, "rb"); if (!f) return false;
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    v.resize(n); bool ok = fread(v.data(), 1, n, f) == (size_t)n; fclose(f); return ok;
}

int main(int argc, char **argv)
{
    if (argc != 4) { fprintf(stderr, "usage: crxdec cmp1.bin sample.bin out.u16\n"); return 2; }
    std::vector<unsigned char> cmp1; std::vector<char> blob; std::vector<unsigned short> out;
    if (!slurp(argv[1], cmp1) || !slurp(argv[2], blob)) { fprintf(stderr, "read error\n"); return 2; }
    CrxH d; int w = 0, h = 0;
    int rc = d.decode(cmp1, blob, out, w, h);
    if (rc) { fprintf(stderr, "decode failed %d\n", rc); return 1; }
    FILE *f = fopen(argv[3], "wb"); fwrite(out.data(), 2, out.size(), f); fclose(f);
    return 0;
}
