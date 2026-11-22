/* R: Debug > Taller 4K (test) (EOS R 1.8.0, v4.61; v4.62 RAM-list check tolerates calibrated values; v4.63 SAP block write (froze, removed); v4.64 cpu1 remap via RPC; v4.65 geometry/ROD + exit trace; v4.66 no-tap variant; v4.68 FROM-blob engine init heights; v4.69/v4.70 revert while magnify is active). Personal diagnostic, not for upstream.
 *
 * First sensor-window write: 4K mode +40 rows (MEM1 4256x2214 -> 4256x2254), notes lv-survey stages 2-5.
 *   1. Checks: movie 4K live view (MEM1 4256x2214 after lv_save_raw), Mem1Path pack cfg at Canon's default, the
 *      SensorDrive table pointer *(0xcfd4 + 0x6c) maps the FROM blob (ROM 0xE1C60000) into RAM (base = ptr - 0x15CE0,
 *      first 64 table bytes equal ROM), the 4K full init list (ROM 0xE1C62028, 113 words) and per-frame LV template
 *      (ROM 0xE1C6288C, 38 words) are word-for-word equal in RAM, regs 0x69 / 0x97 hold 0x6aa / 0x453, and the
 *      MEM1 size record (ROM 0xE0F2E608) is {4256, 2214}. Any mismatch: nothing is written.
 *   2. lv_save_raw(1) + 10-bit packing like the recorder (Mem1Path +0x18 = 3040, f2 = f3 = 0).
 *   3. Patch: record h 2214 -> 2254 (shared ROM remap page 0xE0F2E000), RAM lists 0x69 += 20 (bottom edge, 2-row
 *      units), 0x97 += 20 (output rows / 2). The user then enters and leaves magnify, which rebuilds the 4K path.
 *   4. A 5 ms poll re-narrows +0x18 to 3040 whenever Canon rewrites it with 4256 (rebuild; tap test v4.60 showed port
 *      3 follows 100-200 ms later) and restores 14-bit when the width is anything else (magnify, no 10-bit width).
 *      Even a missed frame at 14-bit overruns MEM1 by only 40 x 7448 B, inside the small-YUV block behind it.
 *   5. Once back in 4K: logs Mem1Path size, live LV list 0x68/0x69 (written directly if the rebuild didn't take the
 *      template), frame interval, stalls; saves rows 2140..2253 and 1100..1103 to ML/LOGS/TALL4K.BIN.
 *   6. Reverts the record and lists, asks for one more magnify in/out (normal 4K path), restores the pack cfg and
 *      lv_save_raw(0). Breadcrumbs: every step is appended to ML/LOGS/TALL4K.TXT and the file closed at once.
 * Opening the ML menu cancels (reverts as in 6). */
#include "dryos.h"
#include "menu.h"
#include "fio-ml.h"
#include "gui.h"
#include "mem.h"
#include "propvalues.h"
#include "property.h"
#include "config.h"
#include "romremap_r.h"

#ifndef MEM
#define MEM(x) (*(volatile uint32_t *)(x))
#endif

#define T4_LOG      "ML/LOGS/TALL4K.TXT"
#define T4_BIN      "ML/LOGS/T4_%d%s.BIN" /* v4.71: one file per height; v4.74: C = centred */
#define T4_PATH     0x133CC
#define T4_STATE    0xef24              /* ImageSequence: +0xc m_SensorMode (12 = 4K, 13 = magnify) */
#define T4_SDRV     0xcfd4
#define T4_UNIT     0xD0422224u
#define T4_MEM1     0x68EEA940u
#define T4_LVP      0x176a8
#define T4_FROM     0xE1C60000u
#define T4_SDTBL    0xE1C75CE0u
#define T4_FULL     0xE1C62028u
#define T4_NFULL    113
#define T4_LVT      0xE1C6288Cu
#define T4_NLVT     38
#define T4_REC      0xE0F2E608u         /* MEM1 record 1 (4K): {w, h, 0, 0} */
#define T4_GEOM     0xE1C75B40u         /* v4.65: FROM-blob readout record for 4K (u16 4256, 2214, 0, 0, 144, 46 ..) */
#define T4_ROD_OFF  (0x410 + 11 * 0x1a)  /* SensorDrive table: ROD u16 for mode 11 (4K), + slot * 2 */
/* v4.82: Canon's LV/YUV crop table (RAM copy of ROM data, privileged RW, shareable): FUN_022a9bf8(mode, variant)
 * returns &table[mode][variant]; sensor mode 11 (4K) -> 4 entries at 0x022fee34, 32 bytes each:
 * {in_w 4256, in_h 2214, out_w 3840, out_h 2144, x 276, y 58, grid 160, 134}. y = 46 OB rows + (2168 - 2144) / 2:
 * centred on the image, so the centred taller frame needs y + rows/2 (and in_h = the real height) for the screen
 * and Canon's video path to show the same centre as the raw. Magnify (mode 16) has its own entries. */
#define T4_YUV      0x022fee34u
/* v4.83: ROM "valid area" records (11 words {in_w, in_h, 1, x, y, w, h, 0, 0, w, h}), two for 4K: 272/47 3848x2166.
 * The 3848 width matches the LV DAF (dual-pixel AF) buffer's line (notes, v5 dump), and camera v4.82 focused on the
 * background with the AF point just inside a near object's top edge at +480 centred: the AF data still came from
 * 240 rows higher. Shifted like the YUV crop (y + rows/2, in_h = real height) through the ROM remap. */
static const uint32_t t4_va[2] = { 0xE12250B8, 0xE12251C0 };
static uint32_t *t4s_var[2];
/* v5.06 (desk D1, wider-desk.md): the LV path's line width is valid-area word 0 (in_w): MvRawProPath_SetPath writes the
 * record into the LTKIDS crop block (0xD041E088.. = (in_h-1) << 16 | (in_w/2 - 1)). Wide left it at 4256 for a 4480 line:
 * each LV row started 224 px late -> sheared, Bayer-flipped horizontal bands. 0 off, 1 in_w only, 2 in_w + window x
 * moved by the extra left columns (frames like normal 4K). */
CONFIG_INT("t4.wlv", t4_wlv, 2);
/* v4.89: per-mode table 0xE1224CC8 (20-byte entries {flag, 0, valid area*, out*, out*}) pairs each valid-area record
 * with a 17-word output record; 4K: 0xE1225288 / 0xE1225354 {4, 3, 3840, 2160, 1, 1, 1, 1, 0x10001, 3840, 0, 0,
 * 3840, 2160, 1, 1, 0} (hypothesis: the LV YUV / display picture the valid area is resized to).
 * Debug > Taller 4K tests > Taller 4K preview (not saved): Centre = v4.84 (16:9 centre of the frame on screen);
 * Squeeze = valid area covers the whole picture (h 2166 + rows, y 47): whole frame on screen, squeezed vertically;
 * Fit = Squeeze + output width [2] and [12] narrowed to keep the aspect (3840 * 2166 / (2166 + rows), /16). */
static const uint32_t t4_vo[2] = { 0xE1225288, 0xE1225354 };
static uint32_t *t4s_vout[2];
int t4_preview;                         /* v4.89 preview test, R_TESTS only (Debug > Taller 4K tests): 0 Centre, 1 Squeeze,
                                         * 2 Fit. Camera: Squeeze only moved the 2166-row LV window up (valid-area h is
                                         * ignored, v4.91 photos), Fit froze (v4.90). v4.92: always Centre otherwise. */
static int t4_preview_run;              /* preview the held frame was applied with */
/* v5.10 (desk D2 / D3, wider-desk.md): the screen picture is NOT made from the output record above. MvDevelop develops
 * a fixed 3848x2166 from the raw in MEMNAVI 0x10012 (own record 0xE1239C58), and the only resizer on the way to the
 * screen is Opti in MvPostDevelop: per-mode entries 0xE11D411C + e * 0x124, +0x90 -> {k_h, k_v} record, input span =
 * out * (1024 - k) / 1024, read on every frame-parameter update (FUN_e06d0224). So Squeeze wrote 2166 + rows into the
 * 14,585,844-byte raw buffer (12-bit: ~687 KB past its end per frame at +480) and the developer showed the top 2166
 * rows: v5.10 forces Centre in every build.
 * Screen zoom proof (Debug > Taller 4K tests, not saved): the 4K record 0xE11D3DFC {0, 0} -> {103, 103}, word for word
 * Canon's digital-IS level 1 entry (entry 1 differs from entry 0 only in that pointer). No size or buffer changes.
 * Predicted: screen 1.112x closer about the centre (3454x1943 of the 3840x2160 view), recording unchanged. Proves or
 * disproves that Opti runs on the 4K standby path and obeys a remapped record, before any whole-frame attempt. */
#define T4_PK       0xE11D3DFCu         /* MvPostDevelop k record of entry 0 (4K), followed by {103, 103}, {308, 308} */
#define T4_PKPTR    0xE11D41ACu         /* entry 0 +0x90 -> T4_PK */
#define T4_PDSTATE  0x18D18u            /* MvPostDevelop state: +6 u16 EZoom step, +0x78 entry*, +0x80 / +0x84 per-frame zoom */
#define T4_MDSTATE  0x246E4u            /* MvDevelop state: +0x24 entry* (4K 0xE123957C) */
int t4_pk;                              /* menu: 0 OFF, 1 k = 103 */
static int t4_pk_run;
static uint32_t *t4s_pk;
static void t4_log(const char *fmt, ...);
static void t4_pk_log(const char *when)
{
    t4_log("zoom proof %s: k ROM view %d %d, PostDev entry %x step %d zoom %d %d, MvDev entry %x\n", when, (int)MEM(T4_PK),
           (int)MEM(T4_PK + 4), (int)MEM(T4_PDSTATE + 0x78), (int)*(volatile uint16_t *)(T4_PDSTATE + 6),
           (int)MEM(T4_PDSTATE + 0x80), (int)MEM(T4_PDSTATE + 0x84), (int)MEM(T4_MDSTATE + 0x24));
}

#define T4_NYUV     4
/* v4.68: 4K engine init values in the FROM blob ({reg, value} lists) that carry the frame height; v4.64-v4.67 left
 * them at 2214, incl. the 0xD0411xxx block (sensor input side?) at h+1 = 2215. Value address (ROM), Canon's value. */
static const struct { uint32_t a, v; } t4_eng[] = {
    { 0xE1C680BC, 0x08A7109C }, { 0xE1C680CC, 0x08A7109C }, { 0xE1C680D4, 0x08A70002 }, { 0xE1C68994, 0x000008A7 },
    { 0xE1D7E084, 0x08A5213C }, { 0xE1D7E09C, 0x08A6213E }, { 0xE1C6A1C0, 0x08A5109E }, { 0xE1C6A1E8, 0x08A5008E },
    { 0xE1C6A89C, 0x08A5109E }, { 0xE1C6AF70, 0x08A5109E }, { 0xE1C6AF78, 0x08A5002E },
};
#define T4_NENG (sizeof(t4_eng) / sizeof(t4_eng[0]))
/* v4.71: extra rows selectable (sensor regs count 2-row units). Limit: 23.98p VD 1444 HD, ROD = 1110 + rows/2 must
 * stay <= RPS - 10 (1399) and VD - 45 (Canon's own tightest gap, 4K 29.97p) -> <= +578 rows. */
static int t4_sel;                      /* menu: 0 +40, 1 +200, 2 +400, 3 +480, 4 +560 */
static const int t4_steps[5] = { 40, 200, 400, 480, 560 };
static int t4_drows = 40;
static int t4_place = 1;                /* v4.88: centred by default (the mode's placement) */
static int t4_hold;                     /* v4.75 menu: 0 revert after the test, 1 hold the taller frame for recording */
static volatile int t4_holding;         /* v4.75: taller 4K path up, owned by this test (rectest_r.c asks) */
int tall4k_active(void) { return t4_holding; }                    /* v4.74 menu: 0 bottom (start reg 0x68 unchanged), 1 centred */
static int t4_d68, t4_d69;              /* change of 0x68 / 0x69 in 2-row units */
#define T4_DROWS    ((uint32_t)t4_drows)
#define T4_W        4256
#define T4_H        2214
#define T4_WSET     3040                /* 10-bit width field: (3040 * 14) >> 3 = 5320 = 4256 * 10 / 8 */
#define T4_PITCH    5320
static uint32_t t4_fit_w(void) { return (3840 * 2166 / (2166 + T4_DROWS)) & ~31u; }   /* v4.89 Fit preview width (v4.90: /32, half-size record gets /16) */
static uint32_t t4_eng_new(uint32_t v) { return v < 0x10000 ? v + T4_DROWS : v + (T4_DROWS << 16); }
/* v4.93 wider 4K test (desk stage W1, notes/wider-desk.md, ROM values re-checked): extra columns W (multiple of 8),
 * centred. Sensor 0x74/0x75 = image window start/end, pos = (v >> 8) * 48 + (v & 0xff), 4 px units (4K 378..1403);
 * line time fixed (HD 28.88 us): HD/2 = w/8 + blank + 23, so the blank (0xD0411188 = 167, sensor 0x60 = 0x10000 |
 * (2 * blank + 6)) shrinks by W/8. Width-carrying engine values below; MEM1 record w and readout record w. */
static int t4_dcols;                    /* 0 = taller tests / mode; > 0 only from Debug > Wider 4K, no raw tap */
#define T4_DCOLS    ((uint32_t)t4_dcols)
/* v4.96 longer line (desk stage W2, notes/wider-desk.md, ROM re-checked): camera v4.93 froze with the blank cut 167 -> 159.
 * HD +16 (1444 -> 1460 x 20 ns) gives room for +64 cols at Canon's blank 167 (HD/2 = w/8 + blank + 23); 23.98p kept with
 * VD 1428, RPS 1393 (= VD - 35), FA28 1610 (frame ticks = VD * HD + FA28 / 3 = 1001/24000 s exactly). HD reaches the
 * hardware through the TG list 0xD0406198/619C (= HD-1) and sensor 0x51 (= 0x2000 | (HD/2-1)); VD is derived from the
 * SensorDrive table at run time; AE keeps the line time in ns (RAM 0x022EDEF8 for mode 11). Slot 4 (23.98) only. */
static int t4_dhd;                      /* 0 or 16 */
/* v5.00: left part of the extra columns in 4-px units (0x74 start); -1 = half. Canon's 4K start 378 * 4 = 1512 px is a
 * multiple of 12 and v4.98/v4.99 (start 370 / 371) came out with every line shuffled in a 12-pixel pattern. */
static int t4_lunits = -1;
/* v5.01 (desk W4): sensor 0x76 = column region select (4K 0xff = centre ~4128 px; magnify L 0xaa / R 0x55; every
 * full-width mode 0x00). Camera v4.98-v5.00: whatever 0x74/0x75 say, only sensor cols 1502..5627 carry image.
 * -1 = leave Canon's. */
static int t4_r76 = -1;
static int t4s_i76f = -1, t4s_i76t = -1; static uint32_t t4s_o76f, t4s_o76t;
#define T4_LU       ((uint32_t)(t4_lunits >= 0 ? t4_lunits : (int)(T4_DCOLS / 8)))
static int t4_runits = -1;             /* v5.02: right part in 4-px units; -1 = the rest of DCOLS */
#define T4_RU       ((uint32_t)(t4_runits >= 0 ? t4_runits : (int)(T4_DCOLS / 4 - T4_LU)))
/* v5.02: sensor columns actually added (window), vs T4_DCOLS = MEM1 / receiver width (+8 pad cols as in 4K) */
#define T4_WINC     (4 * (T4_LU + T4_RU))
/* v5.02: frame timing per line-time step (frame ticks = VD * HD + FA28 / 3 = 1001/24000 s; RPS = VD - 35) */
#define T4_VD       (T4_DHD == 56 ? 1390 : 1428)
#define T4_RPS      (T4_VD - 35)
#define T4_FA28     (T4_DHD == 56 ? 1250 : 1610)
/* v4.98: tapped wider frame: width WN, 10-bit line = WN * 10 / 8 bytes, Mem1Path width field such that
 * (field * 14) >> 3 = line bytes (4256 -> 3040 / 5320, 4320 -> 3086 / 5400) */
#define T4_WN       (T4_W + T4_DCOLS)
#define T4_PITCHN   (T4_WN * 10 / 8)
#define T4_WSETN    ((T4_PITCHN * 8 + 13) / 14)
/* v5.03: recorder hooks (rectest_r.c): held frame width in px and the 10-bit width field Mem1Path +0x10/+0x18 shows */
int tall4k_width(void) { return t4_holding ? (int)T4_WN : T4_W; }
int tall4k_wfield(void) { return t4_holding ? (int)T4_WSETN : (int)T4_WSET; }
#define T4_DHD      ((uint32_t)t4_dhd)
#define T4_TG98     0xE1C672ECu          /* FROM: value of reg 0xD0406198 (HD-1) */
#define T4_TG9C     0xE1C672F4u          /* FROM: value of reg 0xD040619C (HD-1) */
#define T4_AELINE   0x022EDEF8u          /* AE line time ns, mode 11 (RAM-code data, ROM 0xE139A1B4) */
static int t4s_i51f = -1; static uint32_t t4s_o51f;
static uint32_t t4_hpos(int pos) { return ((uint32_t)(pos / 48) << 8) | (uint32_t)(pos % 48); }
/* kind: 1 low half + W, 2 low half + 2W, 3 value + W/8, 4 value - W/8, 5 high half + W */
static const struct { uint32_t a, v; int kind; } t4_weng[] = {
    { 0xE1C680BC, 0x08A7109C, 1 }, { 0xE1C680CC, 0x08A7109C, 1 }, { 0xE1C6A1C0, 0x08A5109E, 1 },
    { 0xE1C6A89C, 0x08A5109E, 1 }, { 0xE1C6AF70, 0x08A5109E, 1 }, { 0xE1D7E084, 0x08A5213C, 2 },
    { 0xE1D7E09C, 0x08A6213E, 2 }, { 0xE1C689A4, 0x00000213, 3 }, { 0xE1C689AC, 0x000000A7, 4 },
    { 0xE1C6AF88, 0x10980090, 5 },
    /* v4.97 (desk W3, ROM-checked): DIGIC 0xD0510xxx block, per-mode lists sent at every LV start; Canon keeps them
     * consistent with width and blank in every mode (4K / magnify / FHD): B30 = w/4+1, 800..80C = w/4+12,
     * B34 = 2*blank+3, 820..82C = 2*blank+14. v4.93 and v4.96 left them at 4K's values -> no frame. */
    { 0xE1C684B4, 0x00000429, 6 }, { 0xE1C67A64, 0x00000434, 6 }, { 0xE1C67A6C, 0x00000434, 6 },
    { 0xE1C67A74, 0x00000434, 6 }, { 0xE1C67A7C, 0x00000434, 6 },
    { 0xE1C684BC, 0x00000151, 7 }, { 0xE1C67A84, 0x0000015C, 7 }, { 0xE1C67A8C, 0x0000015C, 7 },
    { 0xE1C67A94, 0x0000015C, 7 }, { 0xE1C67A9C, 0x0000015C, 7 },
};
#define T4_NWENG (sizeof(t4_weng) / sizeof(t4_weng[0]))
static uint32_t t4_wdelta(uint32_t a)
{
    for (unsigned k = 0; k < T4_NWENG; k++)
        if (t4_weng[k].a == a)
            switch (t4_weng[k].kind)
            {
                case 1: return T4_DCOLS;
                case 2: return 2 * T4_DCOLS;
                case 3: return T4_DCOLS / 8;
                case 4: return (uint32_t)((int)T4_DHD / 2 - (int)(T4_DCOLS / 8));   /* blank (v4.96: + HD/2 growth) */
                case 5: return T4_WINC << 16;                                                  /* v5.02: window width (W4) */
                case 6: return T4_DCOLS / 4;                                                   /* w/4 + c */
                case 7: return (uint32_t)(2 * ((int)T4_DHD / 2 - (int)(T4_DCOLS / 8)));       /* 2 * blank + c */
            }
    return 0;
}
static int t4s_i74f = -1, t4s_i75f = -1, t4s_i60f = -1, t4s_i74t = -1, t4s_i75t = -1;
static uint32_t t4s_o74f, t4s_o75f, t4s_o60f, t4s_o74t, t4s_o75t;

static volatile int t4_running;
static int t4_tap = 1;           /* v4.66: 0 = no raw tap (no lv_save_raw, no 10-bit, no reflex): A/B for the hang */
static int t4_trace_ms;          /* v4.65: breadcrumb window after the first magnify exit */
/* v4.69: the live 4K list word to restore once magnify is active (lv7 is then not the list being sent) */
static volatile uint32_t *t4_pend_lv;
static uint32_t t4_pend_val;
/* v4.70: everything the revert writes, so it can run while magnify is active (camera v4.69 froze when the
 * init values were restored under the running taller path: some are evidently used per frame) */
static volatile int t4_pend_all;
static volatile int t4_pend_apply, t4_applied, t4_apply_fail;   /* v4.72: apply only while magnify is active */
static void (*t4_restore_fn)(void);
static char t4_result[80];

static void t4_log(const char *fmt, ...)
{
    char b[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    FILE *f = FIO_CreateFileOrAppend(T4_LOG);
    if (f) { FIO_WriteFile(f, b, n); FIO_CloseFile(f); }
}

static int t4_ram_ok(uint32_t a) { return a >= 0x40001000 && a < 0x7f000000 && !(a & 3); }

/* index of the word carrying reg r in a 0xffffffff-terminated list of n words, -1 if none */
static int t4_find(volatile uint32_t *l, int n, int r)
{
    for (int i = 0; i < n; i++) { if (l[i] == 0xffffffff) return -1; if ((l[i] >> 22) == (uint32_t)r) return i; }
    return -1;
}

/* write one list word through the cached alias and clean it, so cached and uncached readers both see it */
static void t4_put(volatile uint32_t *unc, uint32_t v)
{
    uint32_t *c = (uint32_t *)((uint32_t)unc & ~0x40000000);
    *c = v;
    romremap_flush(c, 4);
    if (*unc != v) *unc = v;
}

static volatile uint32_t *t4_live_lv7(void)
{
    uint32_t p = MEM(T4_LVP);
    if (p < 0x1000 || p >= 0x80000000 || (p & 3)) return 0;
    return (volatile uint32_t *)(p + 0x1bac + 7 * 0xac);
}

static uint32_t t4_rowsum(int row)
{
    volatile uint32_t *r = (volatile uint32_t *)(T4_MEM1 + row * T4_PITCH);
    uint32_t h = 0;
    for (int i = 0; i < 256; i += 4) h = h * 31 + r[i];
    return h;
}

/* keep the pack setup consistent with whatever width Canon just wrote; returns 1 if it acted */
static int t4_reflex(int *narrowed, int *reflex14)
{
    if (!t4_tap) return 0;
    /* v4.98: the wider rebuild writes +18 = WN (4320): narrow to its own 10-bit field; normal 4K keeps 4256 -> 3040 */
    uint32_t w18 = MEM(T4_PATH + 0x18), f2 = MEM(T4_PATH + 0x6c);
    int is4k = w18 == T4_W || (T4_DCOLS && w18 == T4_WN);
    uint32_t wset = w18 == T4_W ? T4_WSET : T4_WSETN;
    int is10 = w18 == T4_WSET || (T4_DCOLS && w18 == T4_WSETN);
    if (is4k && f2 == 0)                                /* 4K rebuild: re-narrow for 10-bit */
    {
        uint32_t o = cli(); MEM(T4_PATH + 0x18) = wset; sei(o);
        (*narrowed)++;
        return 1;
    }
    if (!is4k && !is10 && f2 == 0)                      /* other mode (magnify): no 10-bit width -> 14-bit */
    {
        uint32_t o = cli(); MEM(T4_PATH + 0x6c) = 2; MEM(T4_PATH + 0x70) = 1; sei(o);
        (*reflex14)++;
        return 1;
    }
    if (is4k && f2 == 2)                                /* back in 4K at 14-bit: 10-bit */
    {
        uint32_t o = cli(); MEM(T4_PATH + 0x18) = wset; MEM(T4_PATH + 0x6c) = 0; MEM(T4_PATH + 0x70) = 0; sei(o);
        (*narrowed)++;
        return 1;
    }
    return 0;
}

/* wait for a magnify-in/-out cycle (m_SensorMode 13 then 12) while running the reflex; 0 = done, 1 = cancel, 2 = timeout */
static uint32_t t4s_base; static volatile uint32_t *t4s_full, *t4s_lvt; static int t4s_i69f, t4s_i97f, t4s_i69t;
static uint32_t t4s_o69f, t4s_o97f, t4s_o69t, t4s_orec_h; static int t4s_i68f = -1, t4s_i68t = -1; static uint32_t t4s_o68f, t4s_o68t; static uint32_t *t4s_rec_ram; static volatile uint16_t *t4s_geo, *t4s_rod;
static const uint8_t t4s_rod_slots[4] = { 2, 3, 4, 9 };
static int t4s_slot;
static uint32_t t4s_sd;                 /* v4.96: SensorDrive table (RAM) */
static void t4_sdw(uint32_t field, uint32_t v)
{
    uint16_t *c = (uint16_t *)((t4s_sd + field + 11 * 0x1a + 4 * 2) & ~0x40000000);
    *c = v; romremap_flush(c, 2);
}
static void t4_apply_all(void)
{
    t4s_rec_ram[1] = T4_H + T4_DROWS; romremap_flush(&t4s_rec_ram[1], 4);
    t4_put(&t4s_full[t4s_i69f], t4s_o69f + t4_d69);
    t4_put(&t4s_full[t4s_i97f], t4s_o97f + T4_DROWS / 2);
    t4_put(&t4s_lvt[t4s_i69t], t4s_o69t + t4_d69);
    if (t4_d68) { t4_put(&t4s_full[t4s_i68f], t4s_o68f - t4_d68); t4_put(&t4s_lvt[t4s_i68t], t4s_o68t - t4_d68); }
    uint16_t *gc = (uint16_t *)((uint32_t)&t4s_geo[1] & ~0x40000000);
    *gc = T4_H + T4_DROWS; romremap_flush(gc, 2);
    for (int k = 0; k < 4; k++)
    {
        uint16_t *rc2 = (uint16_t *)((uint32_t)&t4s_rod[t4s_rod_slots[k]] & ~0x40000000);
        if (t4s_rod_slots[k] == t4s_slot || T4_DROWS <= 40) { *rc2 = 1110 + T4_DROWS / 2; romremap_flush(rc2, 2); }
    }
    for (unsigned k = 0; k < T4_NENG; k++)
        t4_put((volatile uint32_t *)(t4s_base + (t4_eng[k].a - T4_FROM)), t4_eng_new(t4_eng[k].v) + t4_wdelta(t4_eng[k].a));
    if (T4_DCOLS || T4_DHD)
    {
        /* v4.93: width-only engine values (the shared ones got their delta above), records, sensor window + blank */
        for (unsigned k = 0; k < T4_NWENG; k++)
        {
            int shared = 0;
            for (unsigned j = 0; j < T4_NENG; j++) if (t4_eng[j].a == t4_weng[k].a) shared = 1;
            if (!shared) t4_put((volatile uint32_t *)(t4s_base + (t4_weng[k].a - T4_FROM)), t4_weng[k].v + t4_wdelta(t4_weng[k].a));
        }
        t4s_rec_ram[0] = T4_W + T4_DCOLS; romremap_flush(&t4s_rec_ram[0], 4);
        uint16_t *gw = (uint16_t *)((uint32_t)&t4s_geo[0] & ~0x40000000);
        *gw = T4_W + T4_DCOLS; romremap_flush(gw, 2);
        t4_put(&t4s_full[t4s_i74f], (t4s_o74f & ~0x3fffff) | t4_hpos(378 - T4_LU));
        t4_put(&t4s_full[t4s_i75f], (t4s_o75f & ~0x3fffff) | t4_hpos(1403 + T4_RU));
        t4_put(&t4s_lvt[t4s_i74t], (t4s_o74t & ~0x3fffff) | t4_hpos(378 - T4_LU));
        t4_put(&t4s_lvt[t4s_i75t], (t4s_o75t & ~0x3fffff) | t4_hpos(1403 + T4_RU));
        t4_put(&t4s_full[t4s_i60f], (t4s_o60f & ~0x3fffff) | (0x10000 | (2 * (167 + T4_DHD / 2 - T4_DCOLS / 8) + 6)));
        if (t4_r76 >= 0)
        {
            t4_put(&t4s_full[t4s_i76f], (t4s_o76f & ~0x3fffff) | (uint32_t)t4_r76);
            t4_put(&t4s_lvt[t4s_i76t], (t4s_o76t & ~0x3fffff) | (uint32_t)t4_r76);
        }
    }
    if (T4_DHD)
    {
        /* v4.96: line time + frame length (all inactive until the magnify exit rebuilds 4K) */
        t4_put((volatile uint32_t *)(t4s_base + (T4_TG98 - T4_FROM)), 0x5a3 + T4_DHD);
        t4_put((volatile uint32_t *)(t4s_base + (T4_TG9C - T4_FROM)), 0x5a3 + T4_DHD);
        t4_put(&t4s_full[t4s_i51f], (t4s_o51f & ~0x3fffff) | (0x2000 | ((1444 + T4_DHD) / 2 - 1)));
        t4_sdw(0x820, 1444 + T4_DHD); t4_sdw(0, T4_VD); t4_sdw(0x618, T4_RPS); t4_sdw(0xa28, T4_FA28);
        t4_put((volatile uint32_t *)T4_AELINE, 20 * (1444 + T4_DHD));
    }
    for (int k = 0; k < T4_NYUV; k++)          /* v4.82: Canon's display crop follows the centred window */
    {
        volatile uint32_t *e = (volatile uint32_t *)(T4_YUV + k * 0x20);
        t4_put(&e[1], T4_H + T4_DROWS);
        t4_put(&e[5], 58 + 2 * t4_d68);
    }
    for (int k = 0; k < 2; k++)
        if (t4s_var[k])
        {
            t4s_var[k][1] = T4_H + T4_DROWS;
            if (t4_preview) { t4s_var[k][4] = 47; t4s_var[k][6] = t4s_var[k][10] = 2166 + T4_DROWS; }
            else t4s_var[k][4] = 47 + 2 * t4_d68;
            if (T4_DCOLS && t4_wlv)
            {
                t4s_var[k][0] = T4_WN;
                if (t4_wlv == 2) t4s_var[k][3] = 272 + 4 * T4_LU;
                if (t4_wlv == 3) t4s_var[k][3] = (272 + 4 * T4_LU + 15) & ~15u;   /* v5.07 test: x on 16 px (Canon's 4K 272 = 17 * 16) */
            }
            romremap_flush(t4s_var[k], 44);
            if (t4_preview == 2 && t4s_vout[k]) { t4s_vout[k][2] = t4_fit_w(); t4s_vout[k][12] = t4_fit_w() / t4s_vout[k][5]; romremap_flush(t4s_vout[k], 68); }
        }
    if (t4_pk_run && t4s_pk) { t4s_pk[0] = 103; t4s_pk[1] = 103; romremap_flush(t4s_pk, 8); }   /* v5.10 screen zoom proof */
}

static void t4_restore_all(void)
{
    t4s_rec_ram[1] = t4s_orec_h; romremap_flush(&t4s_rec_ram[1], 4);
    t4_put(&t4s_full[t4s_i69f], t4s_o69f); t4_put(&t4s_full[t4s_i97f], t4s_o97f); t4_put(&t4s_lvt[t4s_i69t], t4s_o69t);
    if (t4s_i68f >= 0) { t4_put(&t4s_full[t4s_i68f], t4s_o68f); t4_put(&t4s_lvt[t4s_i68t], t4s_o68t); }
    uint16_t *gc = (uint16_t *)((uint32_t)&t4s_geo[1] & ~0x40000000);
    *gc = T4_H; romremap_flush(gc, 2);
    for (int k = 0; k < 4; k++)
    {
        uint16_t *rc2 = (uint16_t *)((uint32_t)&t4s_rod[t4s_rod_slots[k]] & ~0x40000000);
        *rc2 = 1110; romremap_flush(rc2, 2);
    }
    for (unsigned k = 0; k < T4_NENG; k++)
        t4_put((volatile uint32_t *)(t4s_base + (t4_eng[k].a - T4_FROM)), t4_eng[k].v);
    if (T4_DCOLS || T4_DHD)
    {
        for (unsigned k = 0; k < T4_NWENG; k++)
            t4_put((volatile uint32_t *)(t4s_base + (t4_weng[k].a - T4_FROM)), t4_weng[k].v);
        t4s_rec_ram[0] = T4_W; romremap_flush(&t4s_rec_ram[0], 4);
        uint16_t *gw = (uint16_t *)((uint32_t)&t4s_geo[0] & ~0x40000000);
        *gw = T4_W; romremap_flush(gw, 2);
        t4_put(&t4s_full[t4s_i74f], t4s_o74f); t4_put(&t4s_full[t4s_i75f], t4s_o75f);
        t4_put(&t4s_lvt[t4s_i74t], t4s_o74t); t4_put(&t4s_lvt[t4s_i75t], t4s_o75t);
        t4_put(&t4s_full[t4s_i60f], t4s_o60f);
        if (t4_r76 >= 0) { t4_put(&t4s_full[t4s_i76f], t4s_o76f); t4_put(&t4s_lvt[t4s_i76t], t4s_o76t); }
    }
    if (T4_DHD)
    {
        t4_put((volatile uint32_t *)(t4s_base + (T4_TG98 - T4_FROM)), 0x5a3);
        t4_put((volatile uint32_t *)(t4s_base + (T4_TG9C - T4_FROM)), 0x5a3);
        t4_put(&t4s_full[t4s_i51f], t4s_o51f);
        t4_sdw(0x820, 1444); t4_sdw(0, 1444); t4_sdw(0x618, 1409); t4_sdw(0xa28, 842);
        t4_put((volatile uint32_t *)T4_AELINE, 28880);
    }
    for (int k = 0; k < T4_NYUV; k++)
    {
        volatile uint32_t *e = (volatile uint32_t *)(T4_YUV + k * 0x20);
        t4_put(&e[1], T4_H);
        t4_put(&e[5], 58);
    }
    for (int k = 0; k < 2; k++)
        if (t4s_var[k])
        {
            t4s_var[k][0] = T4_W; t4s_var[k][1] = T4_H; t4s_var[k][3] = 272; t4s_var[k][4] = 47; t4s_var[k][6] = t4s_var[k][10] = 2166;
            romremap_flush(t4s_var[k], 44);
            if (t4s_vout[k]) { t4s_vout[k][2] = 3840; t4s_vout[k][12] = 3840 / t4s_vout[k][5]; romremap_flush(t4s_vout[k], 68); }
        }
    if (t4s_pk) { t4s_pk[0] = 0; t4s_pk[1] = 0; romremap_flush(t4s_pk, 8); }
}

static int t4_nocancel;                 /* v4.75: hold: the ML menu must not end the wait */
static int t4_auto = 1;                 /* v4.77: 1 ML requests magnify in/out itself (v4.88: always; 0 = manual, R_TESTS menu) */
static volatile int t4_end_req;         /* v4.77: menu "End taller 4K" while holding */
/* v4.79 persistent mode (Debug > Taller 4K mode, kept in magic.cfg): a watcher switches 4K 23.98p movie LV to the
 * taller frame by itself and back before anything that could rebuild 4K at another size or rate. */
CONFIG_INT("t4.mode", t4_pmode, 0);       /* 0 OFF, 1 +200 (4256x2414, 16:9 in post), 2 +480 (4256x2694), 3 wide 4480x2214 (v5.03), 4 wide +264 rows 4480x2478 (v5.05) */
static int t4_persist_run;              /* the running t4_task was started by the watcher */
static int t4_pmode_run;
static int t4_wlv_run;                  /* v5.06 */                /* the mode value the running switch was started for */
static int t4_dump = 1;                 /* row dump after the switch (test entries only) */
static volatile int t4_key_pend;        /* key held back until normal 4K is back, then re-sent */
static volatile int t4_soon;            /* v4.86: the watcher is counting down to a switch (REC held back) */
static int t4_gm0;                      /* Canon GUI mode when the hold started (v4.79: idle value unverified) */
static volatile int t4_suspend;
static int t4_c1retry;                  /* v5.08: one automatic retry after a cpu1 check failure */         /* watcher: stop auto switching until the mode is changed (after a failure) */
#define T4_PND "ML/SETTINGS/TALL4K.PND"  /* crash marker: exists only while a switch is in progress */
static void t4_marker(int on)
{
    if (on) { FILE *f = FIO_CreateFile(T4_PND); if (f) { FIO_WriteFile(f, "t4", 2); FIO_CloseFile(f); } }
    else FIO_RemoveFile(T4_PND);
}
/* v4.78: on the R PROP_LV_DISPSIZE is 8 bytes (camera v4.77: ML's length check asserted "correct:8 called:4" and
 * nothing was sent). Canon's LVGMT handler (FUN_e04b46b4, case 0x80050015) takes word 0 = zoom and stores word 1 at
 * its +0x504, passing both to FUN_e02cd6e6; word 1's meaning is unknown, so keep Canon's current value. */
static volatile uint32_t t4_dz[2], t4_dz_len, t4_dz_n;
PROP_HANDLER(PROP_LV_DISPSIZE)
{
    t4_dz_len = len;
    t4_dz[0] = buf[0];
    t4_dz[1] = len >= 8 ? buf[1] : 0;
    t4_dz_n++;
}
static void t4_zoom(int z)
{
    uint32_t v[2] = { (uint32_t)z, t4_dz[1] };
    t4_log("  zoom request %d (current %x/%x, len %d, seen %d; mode %d)\n", z, (int)t4_dz[0], (int)t4_dz[1],
           (int)t4_dz_len, (int)t4_dz_n, (int)MEM(T4_STATE + 0xc));
    if (t4_dz_len != 8 || !t4_dz_n) { t4_log("  zoom: property not seen as 8 bytes: not sent\n"); return; }
    if (t4_persist_run && z != 1) t4_marker(1);     /* v4.79: a switch starts (apply or end) */
    prop_request_change(PROP_LV_DISPSIZE, v, 8);
}
/* v5.11: why Canon didn't magnify. LVGMT's gate FUN_e04b1096 (decompiled 2026-11-22) refuses or defers a zoom request
 * while: continuous shooting, recording, FE lock, flicker detection, LVGMT state 5 (mode change), the AF state object
 * not idle (FUN_e02ee258() != 0, log "Magnify Save (AfSeq)") or a DTS flag +0x520 == 0 ("Magnify Save (AutoK)").
 * Camera v5.09 / v5.10: the property went to 5 but m_SensorMode stayed 12 for 22 s, five switches in a row. Both
 * getters are read-only (a state-object read; a table pointer). */
extern int is_continuous_af(void);
static char t4_nomag[56];
static void t4_nomag_why(void)
{
    int afseq = ((int (*)(void))0xE02EE259)();
    uint32_t d = ((uint32_t (*)(void))0xE04FFF9B)();
    int dok = !(d & 3) && ((d >= 0x1000 && d < 0x40000000) || (d >= 0x40001000 && d < 0x7f000000) || (d >= 0xE0000000 && d < 0xE2000000));
    int autok = dok ? (int)MEM(d + 0x520) : -1;
    int servo = is_continuous_af();
    t4_log("  no magnify: Canon AF state busy %d, AutoK flag %d (table %x), Movie Servo AF %d, dispsize prop %d, GUI mode %d\n",
           afseq, autok, (int)d, servo, (int)t4_dz[0], (int)CURRENT_GUI_MODE);
    /* v5.12: camera v5.11 (2 runs): AF idle, AutoK 1, servo off, and the phone photo shows neither Canon's AF frame nor
     * its magnify icon (both present in the working runs' photos): suspect AF method Face+Tracking. Values unverified
     * on the R, logged to learn them. Short texts: the v5.11 message ran off the screen and wrapped over itself. */
    t4_log("  no magnify: af_mode %x, lv_af_system %x\n", (int)af_mode, (int)lv_af_system);
    snprintf(t4_nomag, sizeof(t4_nomag), "%s", afseq ? (servo ? "Movie Servo AF is on" : "Canon AF is busy")
             : autok == 0 ? "Canon blocks zoom (AutoK)" : "Canon refused zoom");
}
static int t4_cycle(int timeout_ms, int *narrowed, int *reflex14)
{
    int seen13 = 0, rq5 = 0, rq1 = 0, ending = 0, dlg_warned = 0, mchg = 0;
    uint32_t t0 = get_ms_clock(), t13 = 0, trq = 0, tstop = 0;
    while (get_ms_clock() - t0 < (uint32_t)timeout_ms)
    {
        if (gui_menu_shown() && !t4_nocancel) return 1;
        /* v4.77 auto magnify: in, wait until this cycle's apply/restore has run, then out. While holding, only
         * after "End taller 4K". A request that changes nothing is retried once after 3 s, then given up. */
        if (t4_persist_run && t4_holding && !ending)
        {
            /* v4.86: decided when the menu closes, so a value cycled through and back (SET pressed on the entry to
             * "confirm") keeps the frame; v4.85 ended it on the first change and the next clip came out normal 4K */
            int ch = t4_pmode != t4_pmode_run || t4_preview != t4_preview_run || t4_wlv != t4_wlv_run || t4_pk != t4_pk_run;   /* v4.89: a preview change re-applies too; v5.10: zoom proof */
            if (ch != mchg)
            {
                mchg = ch;
                t4_log(ch ? "  height %d / preview %d changed while held (ends when the menu closes)\n" : "  height %d / preview %d back: stays on\n", t4_pmode, t4_preview);
            }
            if (mchg && !gui_menu_shown())
            {
                ending = 1;
                t4_log("  menu closed: ending for height %d / preview %d\n", t4_pmode, t4_preview);
                NotifyBox(3000, "4K frame height / preview changed: back to normal 4K first...");
            }
        }
        if (t4_end_req) ending = 1;                 /* kept after t4_end_req is cleared at magnify */
        if (t4_holding && !seen13)
        {
            /* v4.79: LV stopped or another sensor mode (photo LV, FHD): the 4K path is not running, so Canon's
             * values can go back at once (the rule is only: never under a running 4K path) */
            int mm = MEM(T4_STATE + 0xc);
            if (!lv || (mm != 12 && mm != 13))
            {
                if (!tstop) tstop = get_ms_clock();
                else if (get_ms_clock() - tstop > 300)
                {
                    if (t4_pend_all && t4_restore_fn) { t4_restore_fn(); t4_pend_all = 0; }
                    t4_holding = 0;
                    t4_log("  4K path stopped while held (lv %d, mode %d): Canon's values restored\n", lv, mm);
                    return 3;
                }
            }
            else tstop = 0;
            /* a Canon dialog opened anyway (e.g. touch Q): try to end via magnify, log what happens */
            if (CURRENT_GUI_MODE != t4_gm0 && !gui_menu_shown() && !dlg_warned)
            {
                dlg_warned = 1;
                t4_log("  Canon dialog %d opened while held: ending taller 4K\n", (int)CURRENT_GUI_MODE);
                NotifyBox(5000, "Taller 4K: close this screen, back to normal 4K first");
                ending = 1;
            }
        }
        int autonow = ending || (t4_auto && !t4_holding);
        if (autonow && !seen13 && (!rq5 || (rq5 == 1 && get_ms_clock() - trq > 3000)) && !gui_menu_shown())
        { rq5++; trq = get_ms_clock(); t4_zoom(5); }
        if (autonow && !seen13 && rq5 >= 2 && get_ms_clock() - trq > 2500 && !t4_holding && t4_pend_apply)
        {
            /* v5.11: two requests, ~5.5 s, no magnify: give up with Canon's reason instead of waiting 30 s (v5.09 /
             * v5.10: the user powered off mid-wait, which left the crash marker). Nothing was applied yet. */
            t4_nomag_why();
            if (t4_dz[0] != 1) { uint32_t v[2] = { 1, t4_dz[1] }; prop_request_change(PROP_LV_DISPSIZE, v, 8); t4_log("  dispsize back to 1 requested\n"); }
            return 2;
        }
        if (autonow && seen13 && !rq1 && MEM(T4_STATE + 0xc) == 13 && !t4_pend_apply && !t4_pend_all && !t4_pend_lv
            && get_ms_clock() - t13 > 700)
        { rq1 = 1; t4_zoom(1); }
        if (t4_reflex(narrowed, reflex14))
            t4_log("  +%d ms reflex: mode %d, +18 %d, f2 %d, m1 %dx%d\n", (int)(get_ms_clock() - t0),
                   (int)MEM(T4_STATE + 0xc), (int)MEM(T4_PATH + 0x18), (int)MEM(T4_PATH + 0x6c),
                   (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14));
        int m = MEM(T4_STATE + 0xc);
        if (m == 13) { if (!seen13) { t13 = get_ms_clock(); t4_log("  magnify seen at +%d ms (lv_dispsize %d)\n", (int)(t13 - t0), lv_dispsize); } seen13 = 1; t4_holding = 0; t4_end_req = 0; }
        if (!seen13 && t4_pend_apply)
        {
            static uint32_t hb;
            if (get_ms_clock() - hb >= 1000) { hb = get_ms_clock(); t4_log("  waiting for magnify: +%d ms, mode %d\n", (int)(hb - t0), m); }
        }
        if (m == 13 && t4_pend_apply)
        {
            /* v4.72 (camera v4.71 froze at +560 before magnify, with every value already patched under the running
             * normal 4K path): apply everything only while magnify is active, then check cpu1 sees the record */
            msleep(200);
            if (MEM(T4_STATE + 0xc) != 13) { t4_log("  magnify left within 200 ms: not applied\n"); t4_pend_apply = 0; continue; }
            t4_apply_all();
            t4_applied = 1; t4_pend_apply = 0;
            uint32_t h0 = t4s_rec_ram[1], hrom0 = MEM(T4_REC + 4);   /* v5.08: record right after the apply */
            int c1 = -1, calls = 0; uint32_t v1 = 0;
            int r1 = romremap_cpu1(T4_REC + 4, &c1, &v1, &calls);
            /* v5.08: the probe result gets its own line (the long line below is cut at 200 chars, so the v5.06/v5.07
             * refusals had no rc / cpu / h in the log) */
            t4_log("  cpu1 probe: rc %x cpu %d calls %d h %d (want %d); record after apply RAM %d / ROM view %d, now RAM %d / ROM %d\n",
                   r1, c1, calls, (int)v1, T4_H + T4_DROWS, (int)h0, (int)hrom0, (int)t4s_rec_ram[1], (int)MEM(T4_REC + 4));
            if (T4_DCOLS || T4_DHD)
                t4_log("  D0510: B30 %d B34 %d 800 %d 820 %d; D0411158 %x D0411188 %x; 0x60 %x 0x51 %x; TG %x\n",
                       (int)MEM(t4s_base + (0xE1C684B4 - T4_FROM)), (int)MEM(t4s_base + (0xE1C684BC - T4_FROM)),
                       (int)MEM(t4s_base + (0xE1C67A64 - T4_FROM)), (int)MEM(t4s_base + (0xE1C67A84 - T4_FROM)),
                       (int)MEM(t4s_base + (0xE1C689A4 - T4_FROM)), (int)MEM(t4s_base + (0xE1C689AC - T4_FROM)),
                       (int)t4s_full[t4s_i60f], (int)(t4s_i51f >= 0 ? t4s_full[t4s_i51f] : 0),
                       (int)MEM(t4s_base + (T4_TG98 - T4_FROM)));
            t4_log("  magnify active: applied +%d rows; record %d (RAM %d), YUV crop h %d y %d, AF area h %d y %d (area h %d, out w %d), full 0x68 %x 0x69 %x 0x97 %x, template 0x68 %x 0x69 %x, "
                   "ROD %d; cpu1 probe rc %x cpu %d h %d\n", T4_DROWS, (int)MEM(T4_REC + 4), (int)t4s_rec_ram[1],
                   (int)MEM(T4_YUV + 4), (int)MEM(T4_YUV + 20), (int)MEM(t4_va[0] + 4), (int)MEM(t4_va[0] + 16), (int)MEM(t4_va[0] + 24), (int)MEM(t4_vo[0] + 8),
                   (int)t4s_full[t4s_i68f],
                   (int)t4s_full[t4s_i69f], (int)t4s_full[t4s_i97f], (int)t4s_lvt[t4s_i68t], (int)t4s_lvt[t4s_i69t],
                   t4s_rod[t4s_slot], r1, c1, (int)v1);
            if (t4_pk_run && t4s_pk)
            {
                /* v5.10: does cpu1 see the patched k word too? (which core runs FUN_e06d0224 is unknown) */
                int c2 = -1, calls2 = 0; uint32_t v2 = 0;
                int r2 = romremap_cpu1(T4_PK, &c2, &v2, &calls2);
                t4_log("  zoom proof applied: k RAM %d %d, ROM view cpu0 %d, cpu1 %d (rc %x cpu %d)\n", (int)t4s_pk[0], (int)t4s_pk[1],
                       (int)MEM(T4_PK), (int)v2, r2, c2);
                t4_pk_log("in magnify");
            }
            if (r1 || v1 != T4_H + T4_DROWS)
            {
                t4_restore_all(); t4_applied = 0; t4_apply_fail = 1;
                t4_log("  cpu1 does not see the record: restored (still in magnify)\n");
                NotifyBox(5000, "Taller 4K: cpu1 check failed, exit magnify (normal 4K)");
            }
            else if (!t4_auto) NotifyBox(5000, "Taller 4K +%d: now exit magnify", T4_DROWS);
        }
        if (m == 13 && t4_pend_all && t4_restore_fn)
        {
            msleep(200);
            t4_restore_fn();
            t4_pend_all = 0;
            t4_log("  magnify active: all values restored\n");
        }
        if (m == 13 && t4_pend_lv)
        {
            /* v4.69 (camera v4.68 froze when the live 4K list went back to 2214 rows while the taller path ran):
             * restore it only now, while magnify's own list is the one being sent */
            msleep(200);
            t4_put(t4_pend_lv, t4_pend_val);
            t4_log("  magnify active: live 4K list 0x69 restored -> %x\n", (int)*t4_pend_lv);
            t4_pend_lv = 0;
        }
        if (seen13 && m == 12 && t4_trace_ms)
        {
            /* v4.65: magnify exit seen: breadcrumb every 50 ms (file closed each time) while running the reflex,
             * so a hang shows how far the taller path got */
            uint32_t t1 = get_ms_clock(), tl = 0, s0 = t4_rowsum(1100), nchg = 0;
            while (get_ms_clock() - t1 < (uint32_t)t4_trace_ms)
            {
                t4_reflex(narrowed, reflex14);
                uint32_t now = get_ms_clock(), sr = t4_rowsum(1100);
                if (sr != s0) { s0 = sr; nchg++; }
                if (now - tl >= 50)
                {
                    tl = now;
                    t4_log("  trace +%d: mode %d m1 %dx%d +18 %d +1c %d f2 %d unit %x frames %d (tap %d)\n", (int)(now - t1),
                           (int)MEM(T4_STATE + 0xc), (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14),
                           (int)MEM(T4_PATH + 0x18), (int)MEM(T4_PATH + 0x1c), (int)MEM(T4_PATH + 0x6c),
                           (int)MEM(T4_UNIT), (int)nchg, t4_tap);
                }
                msleep(5);
            }
            t4_trace_ms = 0;
        }
        if (seen13 && m == 12 && MEM(T4_PATH + 0x1c) != 1496 && MEM(T4_PATH + 0x14) != 1496)
        {
            /* back in 4K and port 3 reprogrammed: give it 1 s of reflexes to settle */
            uint32_t t1 = get_ms_clock();
            while (get_ms_clock() - t1 < 1000) { t4_reflex(narrowed, reflex14); msleep(5); }
            return 0;
        }
        msleep(5);
    }
    return 2;
}

static void t4_task(void *unused)
{
    int raw = 0, packed = 0, patched = 0, narrowed = 0, reflex14 = 0, rc = 0, rebuilt = 0, held = 0;
    static uint32_t base; uint32_t orig_pack[5] = {0}, w0 = 0;
    base = 0;
    static volatile uint32_t *full, *lvt;
    full = 0; lvt = 0;
    static int i69f, i97f, i69t;
    i69f = i97f = i69t = -1;
    static uint32_t o69f, o97f, o69t, orec_h;
    o69f = o97f = o69t = orec_h = 0;
    static uint32_t *rec_ram;
    rec_ram = 0;
    t4_result[0] = 0;
    if (!t4_persist_run) t4_pk_run = 0;     /* v5.10: the zoom proof only rides on the 4K frame height mode */
    t4_log("=== taller 4K v5.12 %s at %d ms\n", t4_persist_run ? "mode (auto)" : "test", (int)get_ms_clock());
    for (int k = 0; k < 100 && gui_menu_shown(); k++) msleep(100);
    if (gui_menu_shown() || !lv) { snprintf(t4_result, sizeof(t4_result), "menu open or no LV: not started"); goto out; }

    /* 1. checks (read-only) */
    uint32_t sd = MEM(T4_SDRV + 0x6c);
    base = sd - (T4_SDTBL - T4_FROM);
    t4_log("sdrv table ptr %x -> FROM blob base %x\n", (int)sd, (int)base);
    if (!t4_ram_ok(sd) || !t4_ram_ok(base)) { snprintf(t4_result, sizeof(t4_result), "sdrv ptr %x unexpected", (int)sd); goto out; }
    for (int i = 0; i < 16; i++)
        if (MEM(sd + i * 4) != MEM(T4_SDTBL + i * 4))
        { snprintf(t4_result, sizeof(t4_result), "sdrv table != ROM at word %d", i); goto out; }
    full = (volatile uint32_t *)(base + (T4_FULL - T4_FROM));
    lvt = (volatile uint32_t *)(base + (T4_LVT - T4_FROM));
    /* v4.62: camera v4.61 found 13 of 151 words different (calibration written after boot?). Accept the RAM copy
     * when the layout is identical (same register in every word, same terminator), at most 16 values differ, and
     * the words this test patches equal ROM (checked below via the expected values). Log every difference. */
    int bad = 0, layout = 0;
    for (int k = 0; k < 2; k++)
    {
        volatile uint32_t *l = k ? lvt : full;
        uint32_t rom = k ? T4_LVT : T4_FULL;
        int n = k ? T4_NLVT : T4_NFULL;
        for (int i = 0; i <= n; i++)
        {
            uint32_t a = l[i], b = MEM(rom + i * 4);
            if (a == b) continue;
            if ((a >> 22) != (b >> 22) || i == n) layout++;
            else bad++;
            t4_log("  %s[%d] RAM %x ROM %x (reg %x)\n", k ? "template" : "full", i, (int)a, (int)b, (int)(b >> 22));
        }
    }
    t4_log("RAM lists: full %x, LV template %x: %d values differ from ROM, %d layout differences\n", (int)full, (int)lvt,
           bad, layout);
    if (layout || bad > 16)
    { snprintf(t4_result, sizeof(t4_result), "RAM lists != ROM (%d values, %d layout): not touched", bad, layout); goto out; }
    i69f = t4_find(full, T4_NFULL, 0x69); i97f = t4_find(full, T4_NFULL, 0x97); i69t = t4_find(lvt, T4_NLVT, 0x69);
    if (i69f < 0 || i97f < 0 || i69t < 0) { snprintf(t4_result, sizeof(t4_result), "regs not found"); goto out; }
    o69f = full[i69f]; o97f = full[i97f]; o69t = lvt[i69t];
    t4_log("full[%d] = %x, full[%d] = %x, template[%d] = %x\n", i69f, (int)o69f, i97f, (int)o97f, i69t, (int)o69t);
    if ((o69f & 0x3fffff) != 0x6aa || (o97f & 0x3fffff) != 0x453 || (o69t & 0x3fffff) != 0x6aa)
    { snprintf(t4_result, sizeof(t4_result), "reg values unexpected"); goto out; }
    /* v4.74: window start 0x68 (centred placement moves it up by half the extra rows) */
    t4s_i68f = t4_find(full, T4_NFULL, 0x68); t4s_i68t = t4_find(lvt, T4_NLVT, 0x68);
    if (t4s_i68f < 0 || t4s_i68t < 0) { snprintf(t4_result, sizeof(t4_result), "reg 0x68 not found"); goto out; }
    t4s_o68f = full[t4s_i68f]; t4s_o68t = lvt[t4s_i68t];
    t4_d68 = t4_place ? (int)T4_DROWS / 4 : 0;
    t4_d69 = (int)T4_DROWS / 2 - t4_d68;
    t4_log("placement %s: 0x68 full[%d] %x template[%d] %x -> -%d, 0x69 -> +%d (2-row units)\n",
           t4_place ? "centred" : "bottom", t4s_i68f, (int)t4s_o68f, t4s_i68t, (int)t4s_o68t, t4_d68, t4_d69);
    if ((t4s_o68f & 0x3fffff) != 0x26e || (t4s_o68t & 0x3fffff) != 0x26e || (T4_DROWS % 4) || 0x26e - t4_d68 < 0x27)
    { snprintf(t4_result, sizeof(t4_result), "0x68 unexpected (%x): not touched", (int)t4s_o68f); goto out; }
    if (MEM(T4_REC) != T4_W || MEM(T4_REC + 4) != T4_H) { snprintf(t4_result, sizeof(t4_result), "MEM1 record unexpected"); goto out; }
    static volatile uint16_t *geo, *rod;
    geo = (volatile uint16_t *)(base + (T4_GEOM - T4_FROM));
    rod = (volatile uint16_t *)(sd + T4_ROD_OFF);
    t4_log("geometry RAM %x: %d x %d (ROM %d x %d); ROD slots 2/3/4/9: %d %d %d %d\n", (int)geo, geo[0], geo[1],
           *(volatile uint16_t *)T4_GEOM, *(volatile uint16_t *)(T4_GEOM + 2), rod[2], rod[3], rod[4], rod[9]);
    if (geo[0] != T4_W || geo[1] != T4_H || rod[2] != 1110 || rod[3] != 1110 || rod[4] != 1110 || rod[9] != 1110)
    { snprintf(t4_result, sizeof(t4_result), "geometry/ROD unexpected: not touched"); goto out; }
    /* v4.71: timing check for the chosen height in the active fps slot (SensorDrive state +0x70) */
    static int slot;
    slot = (int)MEM(T4_SDRV + 0x70);
    {
        int vd = (slot >= 0 && slot < 13) ? *(volatile uint16_t *)(sd + 11 * 0x1a + slot * 2) : 0;
        int rps = (slot >= 0 && slot < 13) ? *(volatile uint16_t *)(sd + 0x618 + 11 * 0x1a + slot * 2) : 0;
        int rodn = 1110 + T4_DROWS / 2;
        t4_log("step +%d rows (h %d): fps slot %d, VD %d, RPS %d, ROD 1110 -> %d (gap %d)\n", T4_DROWS, T4_H + T4_DROWS,
               slot, vd, rps, rodn, vd - rodn);
        if (T4_DHD) t4_log("  with the longer line: VD %d, RPS %d -> gap %d\n", T4_VD, T4_RPS, T4_VD - rodn);
        if ((slot != 4 && slot != 9 && T4_DROWS > 40) || vd - rodn < 45 || rodn > rps - 10
            || (T4_DHD && (T4_VD - rodn < 45 || rodn > T4_RPS - 10)))
        { snprintf(t4_result, sizeof(t4_result), "+%d rows needs 23.98p (slot %d): not touched", T4_DROWS, slot); goto out; }
    }
    {
        int ebad = 0;
        for (unsigned k = 0; k < T4_NENG; k++)
        {
            uint32_t ram = MEM(base + (t4_eng[k].a - T4_FROM)), reg = MEM(base + (t4_eng[k].a - 4 - T4_FROM));
            if (ram != t4_eng[k].v || (reg >> 20) != 0xD04) { ebad++; t4_log("  engine value %x: RAM %x reg %x (want %x)\n", (int)t4_eng[k].a, (int)ram, (int)reg, (int)t4_eng[k].v); }
        }
        for (int k = 0; k < T4_NYUV; k++)
        {
            volatile uint32_t *e = (volatile uint32_t *)(T4_YUV + k * 0x20);
            if (e[0] != T4_W || e[1] != T4_H || e[2] != 3840 || e[3] != 2144 || e[4] != 276 || e[5] != 58)
            { ebad++; t4_log("  YUV crop entry %d: %d %d %d %d %d %d (want 4256 2214 3840 2144 276 58)\n", k, (int)e[0],
                             (int)e[1], (int)e[2], (int)e[3], (int)e[4], (int)e[5]); }
        }
        if (T4_DCOLS || T4_DHD)
        {
            /* v4.93 width test: every value it writes must be Canon's (ROM) value, behind a 0xD04 engine register */
            for (unsigned k = 0; k < T4_NWENG; k++)
            {
                uint32_t ram = MEM(base + (t4_weng[k].a - T4_FROM)), reg = MEM(base + (t4_weng[k].a - 4 - T4_FROM));
                uint32_t want_reg = t4_weng[k].kind >= 6 ? 0xD05 : 0xD04;
                if (ram != t4_weng[k].v || (reg >> 20) != want_reg) { ebad++; t4_log("  width value %x: RAM %x reg %x (want %x)\n", (int)t4_weng[k].a, (int)ram, (int)reg, (int)t4_weng[k].v); }
            }
            t4s_i74f = t4_find(full, T4_NFULL, 0x74); t4s_i75f = t4_find(full, T4_NFULL, 0x75); t4s_i60f = t4_find(full, T4_NFULL, 0x60);
            t4s_i74t = t4_find(lvt, T4_NLVT, 0x74); t4s_i75t = t4_find(lvt, T4_NLVT, 0x75);
            if (t4s_i74f < 0 || t4s_i75f < 0 || t4s_i60f < 0 || t4s_i74t < 0 || t4s_i75t < 0) { ebad++; t4_log("  width regs not found\n"); }
            else
            {
                t4s_o74f = full[t4s_i74f]; t4s_o75f = full[t4s_i75f]; t4s_o60f = full[t4s_i60f];
                t4s_o74t = lvt[t4s_i74t]; t4s_o75t = lvt[t4s_i75t];
                if ((t4s_o74f & 0x3fffff) != 0x72a || (t4s_o75f & 0x3fffff) != 0x1d0b || (t4s_o60f & 0x3fffff) != 0x10154
                    || (t4s_o74t & 0x3fffff) != 0x72a || (t4s_o75t & 0x3fffff) != 0x1d0b)
                { ebad++; t4_log("  width regs unexpected: 74 %x 75 %x 60 %x / 74 %x 75 %x\n", (int)t4s_o74f, (int)t4s_o75f, (int)t4s_o60f, (int)t4s_o74t, (int)t4s_o75t); }
            }
            if (t4_r76 >= 0)
            {
                t4s_i76f = t4_find(full, T4_NFULL, 0x76); t4s_i76t = t4_find(lvt, T4_NLVT, 0x76);
                t4s_o76f = t4s_i76f >= 0 ? full[t4s_i76f] : 0; t4s_o76t = t4s_i76t >= 0 ? lvt[t4s_i76t] : 0;
                if (t4s_i76f < 0 || t4s_i76t < 0 || (t4s_o76f & 0x3fffff) != 0xff || (t4s_o76t & 0x3fffff) != 0xff)
                { ebad++; t4_log("  0x76 unexpected: full[%d] %x template[%d] %x\n", t4s_i76f, (int)t4s_o76f, t4s_i76t, (int)t4s_o76t); }
                else t4_log("0x76 plan: full[%d] / template[%d] 0xff -> %x\n", t4s_i76f, t4s_i76t, t4_r76);
            }
            int sl = (int)MEM(T4_SDRV + 0x70);
            if (T4_DCOLS && t4_lunits >= 0 && (((378 - T4_LU) % 3) || T4_WINC > T4_DCOLS))
            { ebad++; t4_log("  window start %d..%d not on a 12-px group (or wider than +%d)\n", 378 - T4_LU, 1403 + T4_RU, T4_DCOLS); }
            if ((T4_DCOLS % 8) || T4_DCOLS > 448 || (sl != 4 && sl != 9)) { ebad++; t4_log("  width +%d needs a multiple of 8 <= 448 and 23.98/24p (slot %d)\n", T4_DCOLS, sl); }
            if (T4_DHD)
            {
                /* v4.96 timing: Canon's values, slot 4 (23.98) only, blank must not shrink below 167 */
                volatile uint16_t *t = (volatile uint16_t *)(sd + 11 * 0x1a + 4 * 2);
                uint32_t tg98 = MEM(base + (T4_TG98 - T4_FROM)), tg9c = MEM(base + (T4_TG9C - T4_FROM));
                uint32_t r98 = MEM(base + (T4_TG98 - 4 - T4_FROM)), r9c = MEM(base + (T4_TG9C - 4 - T4_FROM));
                t4s_i51f = t4_find(full, T4_NFULL, 0x51);
                t4s_o51f = t4s_i51f >= 0 ? full[t4s_i51f] : 0;
                t4_log("timing: slot %d, VD %d SetP %d ROD %d RPS %d HD %d FA28 %d, TG %x=%x %x=%x, 0x51 %x, AE line %d ns\n", sl,
                       t[0], t[0x104], t[0x208], t[0x30c], t[0x410], t[0x514], (int)r98, (int)tg98, (int)r9c, (int)tg9c, (int)t4s_o51f,
                       (int)MEM(T4_AELINE));
                if (sl != 4 || t[0] != 1444 || t[0x208] != 1110 || t[0x30c] != 1409 || t[0x410] != 1444 || t[0x514] != 842
                    || r98 != 0xD0406198 || r9c != 0xD040619C || tg98 != 0x5a3 || tg9c != 0x5a3 || t4s_i51f < 0
                    || (t4s_o51f & 0x3fffff) != 0x22d1 || MEM(T4_AELINE) != 28880 || (T4_DHD != 16 && T4_DHD != 56)
                    || 167 + T4_DHD / 2 < 167 + T4_DCOLS / 8)
                { ebad++; t4_log("  timing values unexpected (or not 23.98p): not touched\n"); }
                else t4_log("timing plan: HD 1444 -> %d (TG %x, 0x51 %x), VD %d, RPS %d, FA28 %d, AE %d ns, blank %d\n",
                            1444 + T4_DHD, 0x5a3 + T4_DHD, 0x2000 | ((1444 + T4_DHD) / 2 - 1), T4_VD, T4_RPS, T4_FA28, 20 * (1444 + T4_DHD),
                            167 + T4_DHD / 2 - T4_DCOLS / 8);
            }
            t4_log("width +%d cols: MEM1 w %d, 0x74/0x75 %x/%x -> %x/%x, 0x60 %x -> %x (blank 167 -> %d), %d width values\n",
                   T4_DCOLS, T4_W + T4_DCOLS, 0x72a, 0x1d0b, (int)t4_hpos(378 - T4_LU), (int)t4_hpos(1403 + T4_RU),
                   0x10154, 0x10000 | (2 * (167 + T4_DHD / 2 - T4_DCOLS / 8) + 6), 167 + T4_DHD / 2 - T4_DCOLS / 8, (int)T4_NWENG);
        }
        t4_log("engine init values + %d YUV crop entries: %d checked, %d unexpected\n", T4_NYUV, (int)T4_NENG, ebad);
        if (ebad) { snprintf(t4_result, sizeof(t4_result), "engine values unexpected: not touched"); goto out; }
    }
    rc = romremap_init();
    rec_ram = (uint32_t *)romremap_ram(T4_REC);
    for (int k = 0; k < 2; k++)
    {
        uint32_t *v = rc ? 0 : (uint32_t *)romremap_ram(t4_va[k]);
        t4s_var[k] = 0;
        if (!v || v[0] != T4_W || v[1] != T4_H || v[2] != 1 || v[3] != 272 || v[4] != 47 || v[5] != 3848 || v[6] != 2166)
        {
            t4_log("valid-area record %x: %s %d %d %d %d %d %d %d\n", (int)t4_va[k], v ? "unexpected" : "not mapped",
                   v ? (int)v[0] : 0, v ? (int)v[1] : 0, v ? (int)v[2] : 0, v ? (int)v[3] : 0, v ? (int)v[4] : 0,
                   v ? (int)v[5] : 0, v ? (int)v[6] : 0);
            snprintf(t4_result, sizeof(t4_result), "AF area record unexpected: not touched"); goto out;
        }
        t4s_var[k] = v;
        uint32_t *o = (uint32_t *)romremap_ram(t4_vo[k]);
        t4s_vout[k] = 0;
        /* v4.90: the two 4K output records differ: 0xE1225288 {.., 1, 1, 1, 1, 0x10001, 3840, 0, 0, 3840, 2160 ..},
         * 0xE1225354 {.., 1, 2, 1, 2, 0x20002, 1920, 0, 0, 1920, 1080 ..} (half size): v4.89 expected the first layout
         * for both and refused (mode paused). Check sizes / scale per record; Fit scales [12] by [5]. */
        if (!o || o[0] != 4 || o[1] != 3 || o[2] != 3840 || o[3] != 2160 || (o[5] != 1 && o[5] != 2)
            || o[9] * o[5] != 3840 || o[12] * o[5] != 3840 || o[13] * o[5] != 2160)
        {
            t4_log("output record %x: %s %d %d %d %d %d %d %d\n", (int)t4_vo[k], o ? "unexpected" : "not mapped", o ? (int)o[0] : 0,
                   o ? (int)o[1] : 0, o ? (int)o[2] : 0, o ? (int)o[3] : 0, o ? (int)o[9] : 0, o ? (int)o[12] : 0, o ? (int)o[13] : 0);
            snprintf(t4_result, sizeof(t4_result), "LV output record unexpected: not touched"); goto out;
        }
        t4s_vout[k] = o;
    }
    t4_preview = 0;                        /* v5.10: in every build. Squeeze overran Canon's raw buffer (desk D2), Fit froze */
    t4s_pk = (uint32_t *)romremap_ram(T4_PK);
    if (t4_pk_run)
    {
        /* v5.10 screen zoom proof: the record must be Canon's {0, 0} followed by the digital-IS records, and entry 0
         * must point at it. The state words are logged, not checked: a wrong entry only means no visible effect. */
        if (!t4s_pk || t4s_pk[0] || t4s_pk[1] || t4s_pk[2] != 103 || t4s_pk[3] != 103 || t4s_pk[4] != 308 || MEM(T4_PKPTR) != T4_PK)
        {
            t4_log("zoom proof: k record %s %d %d %d %d, entry 0 ptr %x\n", t4s_pk ? "unexpected" : "not mapped", t4s_pk ? (int)t4s_pk[0] : 0,
                   t4s_pk ? (int)t4s_pk[1] : 0, t4s_pk ? (int)t4s_pk[2] : 0, t4s_pk ? (int)t4s_pk[3] : 0, (int)MEM(T4_PKPTR));
            snprintf(t4_result, sizeof(t4_result), "zoom proof: k record unexpected"); goto out;
        }
        t4_pk_log("plan");
        if (MEM(T4_PDSTATE + 0x78) != 0xE11D411C) t4_log("zoom proof: PostDev entry is not 4K entry 0 (digital IS on? 10-bit?): expect no effect\n");
    }
    if (T4_DCOLS) t4_log("wide screen fix %d: valid area in_w %d -> %d, x 272 -> %d\n", t4_wlv, T4_W, t4_wlv ? (int)T4_WN : T4_W,
                         t4_wlv == 2 ? 272 + 4 * (int)T4_LU : t4_wlv == 3 ? (int)((272 + 4 * T4_LU + 15) & ~15u) : 272);
    t4_log("preview: %s (valid area y %d h %d, output w %d)\n", t4_preview == 2 ? "fit" : t4_preview ? "squeeze" : "centre",
           t4_preview ? 47 : 47 + 2 * t4_d68, t4_preview ? 2166 + (int)T4_DROWS : 2166, t4_preview == 2 ? (int)t4_fit_w() : 3840);
    t4_log("romremap %d (%s), record RAM %x\n", rc, romremap_stage(), (int)rec_ram);
    if (rc || !rec_ram) { snprintf(t4_result, sizeof(t4_result), "ROM remap failed (%d)", rc); goto out; }

    /* 2. raw tap + 10-bit (v4.66: skipped in the no-tap variant; MEM1 is then never written) */
    t4_log("variant: %s; tap flags 0x1311c+4 %d, Mem1Path +40 %d, +50 %d\n", t4_tap ? "raw tap + 10-bit" : "NO raw tap",
           (int)MEM(0x1311C + 4), (int)MEM(T4_PATH + 0x40), (int)MEM(T4_PATH + 0x50));
    if ((T4_DCOLS || T4_DHD) && t4_tap && T4_DCOLS != 0 && T4_DCOLS != 64 && T4_DCOLS != 56 && T4_DCOLS != 224) { snprintf(t4_result, sizeof(t4_result), "tapped width test: +56 / +64 only"); goto out; }
    if (!t4_tap && (MEM(0x1311C + 4) || MEM(T4_PATH + 0x40)))
    {
        /* v4.67: camera v4.66 ran "no tap" with the tap on (RAW zebras/histogram keep lv_save_raw on in movie LV) */
        snprintf(t4_result, sizeof(t4_result), "raw tap in use (RAW zebras?): set Global Draw OFF");
        goto out;
    }
    if (t4_tap) { call("lv_save_raw", 1); raw = 1; msleep(500); }
    for (int i = 0; i < 5; i++) orig_pack[i] = MEM(T4_PATH + 0x64 + 4 * i);
    w0 = MEM(T4_PATH + 0x18);
    t4_log("MEM1 %dx%d, +18 %d, mode %d, pack {%d,%d,%d,%d,%d}, m_SensorMode %d\n", (int)MEM(T4_PATH + 0x10),
           (int)MEM(T4_PATH + 0x14), (int)w0, (int)MEM(T4_PATH), (int)orig_pack[0], (int)orig_pack[1], (int)orig_pack[2],
           (int)orig_pack[3], (int)orig_pack[4], (int)MEM(T4_STATE + 0xc));
    if ((t4_tap && (MEM(T4_PATH + 0x10) != T4_W || MEM(T4_PATH + 0x14) != T4_H || w0 != T4_W)) || MEM(T4_STATE + 0xc) != 12)
    { snprintf(t4_result, sizeof(t4_result), "not movie 4K (MEM1 %dx%d)", (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14)); goto out; }
    if (orig_pack[0] != 0 || orig_pack[1] != 1 || orig_pack[2] != 2 || orig_pack[3] != 1 || orig_pack[4] != 0)
    { snprintf(t4_result, sizeof(t4_result), "pack cfg not Canon's default"); goto out; }
    if (t4_tap)
    {
        uint32_t o = cli();
        MEM(T4_PATH + 0x18) = T4_WSET; MEM(T4_PATH + 0x6c) = 0; MEM(T4_PATH + 0x70) = 0;
        sei(o);
        packed = 1;
        msleep(300);
    }
    t4_log("10-bit: MEM1 w %d, unit %x\n", (int)MEM(T4_PATH + 0x10), (int)MEM(T4_UNIT));

    /* 3. v4.72: nothing is patched here (normal 4K is running); everything is applied while magnify is active */
    orec_h = rec_ram[1];
    t4s_base = base; t4s_full = full; t4s_lvt = lvt; t4s_i69f = i69f; t4s_i97f = i97f; t4s_i69t = i69t;
    t4s_o69f = o69f; t4s_o97f = o97f; t4s_o69t = o69t; t4s_orec_h = orec_h; t4s_rec_ram = rec_ram; t4s_geo = geo;
    t4s_rod = rod; t4s_slot = slot; t4s_sd = sd;
    t4_restore_fn = t4_restore_all;
    t4_log("plan: record h %d -> %d, 0x69 %x -> %x, 0x97 %x -> %x, ROD slot %d -> %d, %d engine values; applied in magnify\n",
           T4_H, T4_H + T4_DROWS, (int)o69f, (int)(o69f + t4_d69), (int)o97f, (int)(o97f + T4_DROWS / 2), slot,
           1110 + T4_DROWS / 2, (int)T4_NENG);
    /* cpu1 onto the remap now (record still Canon's: cpu1 must read 2214); the probe after the apply needs 2214+N */
    {
        int c1 = -1, calls = 0; uint32_t v1 = 0;
        t4_log("cpu1 switch via RPC: requesting\n");
        int r1 = romremap_cpu1(T4_REC + 4, &c1, &v1, &calls);
        t4_log("cpu1 switch: rc %x, callee cpu %d, calls %d, cpu1 reads record h %d (want %d)\n", r1, c1, calls, (int)v1, T4_H);
        if (r1 || v1 != T4_H)
        { snprintf(t4_result, sizeof(t4_result), "cpu1 switch failed (rc %x cpu %d h %d)", r1, c1, (int)v1); goto out; }
    }
    t4_applied = 0; t4_apply_fail = 0; t4_pend_apply = 1; t4_nomag[0] = 0;
    t4_trace_ms = t4_tap ? 2000 : 4000;
    if (t4_auto) NotifyBox(5000, "Taller 4K +%d: switching (auto magnify)...", T4_DROWS);
    else NotifyBox(5000, "Taller 4K +%d: magnify x5, then WAIT for the exit prompt", T4_DROWS);

    /* 4. rebuild via magnify in/out */
    if (t4_persist_run) t4_marker(1);
    rc = t4_cycle(30000, &narrowed, &reflex14);
    t4_log("cycle 1: %s, narrowed %d, 14-bit reflex %d\n", rc == 0 ? "done" : rc == 1 ? "cancelled" : "timeout",
           narrowed, reflex14);
    t4_pend_apply = 0;
    patched = t4_applied;
    if (rc) { snprintf(t4_result, sizeof(t4_result), "%s", rc == 1 ? "cancelled before the rebuild" : t4_nomag[0] ? t4_nomag : "no magnify seen"); goto revert; }
    if (!patched) { snprintf(t4_result, sizeof(t4_result), "%s", t4_apply_fail ? "cpu1 check failed: not applied" : "magnify too short: not applied"); goto revert; }
    rebuilt = 1;

    /* 5. measure */
    uint32_t m1w = MEM(T4_PATH + 0x10), m1h = MEM(T4_PATH + 0x14);
    volatile uint32_t *l7 = t4_live_lv7();
    int i68 = l7 ? t4_find(l7, 43, 0x68) : -1, i69 = l7 ? t4_find(l7, 43, 0x69) : -1;
    if (l7)
    {
        int i74 = t4_find(l7, 43, 0x74), i75 = t4_find(l7, 43, 0x75), i76 = t4_find(l7, 43, 0x76);
        t4_log("after rebuild: live lv7 0x76 %x\n", i76 >= 0 ? (int)l7[i76] : -1);
        t4_log("after rebuild: live lv7 0x74 %x 0x75 %x; Mem1Path +10 %d +14 %d +18 %d +1c %d; record w %d (RAM %d)\n",
               i74 >= 0 ? (int)l7[i74] : -1, i75 >= 0 ? (int)l7[i75] : -1, (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14),
               (int)MEM(T4_PATH + 0x18), (int)MEM(T4_PATH + 0x1c), (int)MEM(T4_REC), (int)t4s_rec_ram[0]);
    }
    if (t4_pk_run) t4_pk_log("after rebuild");
    t4_log("after rebuild: MEM1 %dx%d (+18 %d, +1c %d), f2 %d, unit %x, live lv7 0x68 %x 0x69 %x\n", (int)m1w, (int)m1h,
           (int)MEM(T4_PATH + 0x18), (int)MEM(T4_PATH + 0x1c), (int)MEM(T4_PATH + 0x6c), (int)MEM(T4_UNIT),
           i68 >= 0 ? (int)l7[i68] : -1, i69 >= 0 ? (int)l7[i69] : -1);
    if (i69 >= 0 && (l7[i69] & 0x3fffff) == (o69t & 0x3fffff))
    {
        t4_log("live lv7 0x69 still Canon's after the rebuild (not written: v4.72)\n");
    }
    /* v4.63: camera v4.62 showed Canon's rebuild still used h 2214 (the record is read where the cpu0-only remap
     * doesn't reach, presumably cpu1). So program the size ourselves, as Canon's SAP block setup does: find the
     * {engine register, value} pairs that hold the 4K size word (h-1) << 16 | (w/2 - 1) << 1 in the block structs
     * (RAM 0x1af80..0x1b400, written by FUN_e06b92b4), write the +40-row value to register and struct, then
     * Mem1Path +0x1c (port 3 height, copied to +0x14 every frame by FUN_e0638b7a). Only at 10-bit. */
    int np = 0, nscaled = 0, stalled = 0;
    uint32_t *pa[16];
    const uint32_t v_old = ((T4_H - 1) & 0x3fff) << 16 | (((T4_W >> 1) - 1) & 0x3fff) << 1;
    const uint32_t v_new = ((T4_H + T4_DROWS - 1) & 0x3fff) << 16 | (((T4_W >> 1) - 1) & 0x3fff) << 1;
    for (uint32_t a = 0x1af80; a < 0x1b400; a += 4)
    {
        uint32_t r = MEM(a), v = MEM(a + 4);
        if (r < 0xD0400000 || r >= 0xD0500000 || (r & 3)) continue;
        if (v == v_old)
        {
            t4_log("  size pair at %x: reg %x = %x\n", (int)a, (int)r, (int)v);
            if (np < 16) pa[np++] = (uint32_t *)a;
        }
        else if (((v >> 16) & 0x3fff) == T4_H - 1 || ((v >> 16) & 0x3fff) == T4_H)
        {
            nscaled++;
            t4_log("  other pair with h at %x: reg %x = %x (not patched)\n", (int)a, (int)r, (int)v);
        }
    }
    t4_log("size pairs %x: %d found (other h pairs %d); new value %x\n", (int)v_old, np, nscaled, (int)v_new);
    int blocks_ok = np >= 3 && MEM(T4_PATH + 0x6c) == 0 && MEM(T4_PATH + 0x18) == T4_WSET && MEM(T4_PATH + 0x1c) == T4_H
                    && MEM(T4_STATE + 0xc) == 12;
    int blocks_on = 0;
    int rlo = T4_H + T4_DROWS - 14;
    uint32_t s1100 = t4_rowsum(1100), s2240 = t4_rowsum(rlo);
    if (0 && blocks_ok)   /* v4.64: never write the SAP size registers live (v4.63 froze the camera) */
    {
        uint32_t o = cli();
        for (int k = 0; k < np; k++) { uint32_t r = pa[k][0]; MEM(r) = v_new; pa[k][1] = v_new; }
        MEM(T4_PATH + 0x1c) = T4_H + T4_DROWS;
        sei(o);
        blocks_on = 1;
        t4_log("blocks + Mem1Path +1c written (%d regs)\n", np);
    }
    else t4_log("block patch skipped (pairs %d, f2 %d, +18 %d, +1c %d, mode %d)\n", np, (int)MEM(T4_PATH + 0x6c),
                (int)MEM(T4_PATH + 0x18), (int)MEM(T4_PATH + 0x1c), (int)MEM(T4_STATE + 0xc));
    /* frame interval and stall check (row 1100) + do the new rows get written (row 2240), 1.5 s */
    {
        uint32_t t0 = get_ms_clock(), last = s1100, last2 = s2240, lt = t0, nchg = 0, nchg2 = 0, maxgap = 0;
        while (get_ms_clock() - t0 < 1500)
        {
            t4_reflex(&narrowed, &reflex14);
            uint32_t sa = t4_rowsum(1100), sb = t4_rowsum(rlo), now = get_ms_clock();
            if (sa != last) { if (now - lt > maxgap) maxgap = now - lt; last = sa; lt = now; nchg++; }
            if (sb != last2) { last2 = sb; nchg2++; }
            if (t4_tap && now - lt > 1000) { stalled = 1; break; }
            msleep(2);
        }
        t4_log("row 1100 changed %d x, row %d changed %d x in 1.5 s (max gap %d ms)%s; MEM1 %dx%d\n", (int)nchg,
               rlo, (int)nchg2, (int)maxgap, stalled ? " STALLED" : "", (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14));
    }
    uint32_t m1h2 = MEM(T4_PATH + 0x14);
    if (t4_tap && t4_dump && !stalled && m1h2 == T4_H + T4_DROWS && MEM(T4_PATH + 0x6c) == 0)
    {
        /* v4.71: every 8th row of 0..2139 (whole-frame preview) + every row 2140..h-1 (old edge + new rows),
         * written in 16-row chunks (no big buffer) */
        int hh = T4_H + T4_DROWS, nsp = 2140 / 8, nct = hh - 2140, ok = 0;
        uint8_t *buf = fio_malloc(16 * T4_PITCHN);
        char fn[32]; snprintf(fn, sizeof(fn), T4_BIN, T4_DCOLS ? (int)T4_WN : hh, t4_r76 >= 0 ? "R" : T4_DCOLS ? (t4_lunits >= 0 ? "A" : "W") : T4_DHD ? "L" : t4_d68 ? "C" : "");
        FILE *f = buf ? FIO_CreateFile(fn) : 0;
        if (f)
        {
            uint32_t hdr[16] = { 0x4b344c54 /* "TL4K" */, T4_WN, hh, 10, T4_PITCHN, 0, 8, nsp, 2140, nct, 2 /* layout */ };
            FIO_WriteFile(f, hdr, 64);
            int tot = nsp + nct;
            for (int i = 0; i < tot; i += 16)
            {
                int n = MIN(16, tot - i);
                for (int k = 0; k < n; k++)
                {
                    int j = i + k, row = j < nsp ? j * 8 : 2140 + (j - nsp);
                    memcpy(buf + k * T4_PITCHN, (void *)(T4_MEM1 + row * T4_PITCHN), T4_PITCHN);
                }
                if (FIO_WriteFile(f, buf, n * (int)T4_PITCHN) != n * (int)T4_PITCHN) break;
                ok += n;
            }
            FIO_CloseFile(f);
            t4_log("saved %d of %d rows (every 8th of 0..2139, all of 2140..%d) to %s\n", ok, tot, hh - 1, fn);
        }
        else t4_log("row dump: no buffer or file\n");
        if (buf) fio_free(buf);
        snprintf(t4_result, sizeof(t4_result), "MEM1 %dx%d at 10-bit: rows saved", (int)T4_WN, (int)m1h2);
    }
    else if (t4_tap && !stalled && m1h2 == T4_H + T4_DROWS && MEM(T4_PATH + 0x6c) == 0)
        snprintf(t4_result, sizeof(t4_result), "%dx%d on", (int)T4_WN, (int)m1h2);
    else if (!t4_tap && (T4_DCOLS || T4_DHD)) snprintf(t4_result, sizeof(t4_result), "OK: survived +%d cols, HD +%d", T4_DCOLS, T4_DHD);
    else if (!t4_tap) snprintf(t4_result, sizeof(t4_result), "no tap: survived the taller rebuild (+1c %d)", (int)MEM(T4_PATH + 0x1c));
    else snprintf(t4_result, sizeof(t4_result), "%s MEM1 h %d, pairs %d", stalled ? "STALL," : "", (int)m1h2, np);
    if (blocks_on)
    {
        uint32_t o = cli();
        for (int k = 0; k < np; k++) { uint32_t r = pa[k][0]; MEM(r) = v_old; pa[k][1] = v_old; }
        if (MEM(T4_PATH + 0x1c) == T4_H + T4_DROWS) MEM(T4_PATH + 0x1c) = T4_H;
        sei(o);
        blocks_on = 0;
        msleep(200);
        t4_log("blocks reverted: MEM1 %dx%d, row 1100 %s\n", (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14),
               t4_rowsum(1100) != s1100 ? "moving" : "same");
    }

revert:
    /* 6. revert. v4.70: after a taller rebuild nothing is restored under the running taller path (v4.68 froze on
     * the live list, v4.69 on the init values); everything is restored at once while magnify is active, and the
     * magnify exit rebuilds normal 4K from Canon's values. Without a rebuild everything is inactive: restore now. */
    if (patched)
    {
        if (!rebuilt)
        {
            /* applied in magnify but no exit seen (cancel/timeout): the 4K path has not been rebuilt tall */
            t4_restore_all();
            patched = 0;
            t4_log("reverted now (no taller rebuild happened)\n");
        }
        else
        {
            t4_pend_all = 1;
            int hold = t4_hold && t4_tap && !stalled && MEM(T4_PATH + 0x14) == T4_H + T4_DROWS && MEM(T4_PATH + 0x6c) == 0;
            if (t4_persist_run && !hold) t4_suspend = 1;   /* came up wrong: no automatic retries */
            if (hold)
            {
                /* v4.75: keep the taller frame (10-bit, ML order) for Movie > RAW video; the recorder only flips the
                 * bit order for its encoder and leaves the tap on (tall4k_active). Ends with magnify, no time limit. */
                t4_holding = 1; t4_nocancel = 1; t4_gm0 = CURRENT_GUI_MODE; held = 1;
                t4_log("hold: Canon GUI mode %d\n", t4_gm0);
                t4_log("HOLD: taller 4K %dx%d stays on for recording; magnify x5 + exit ends it\n", (int)T4_WN, T4_H + T4_DROWS);
                t4_end_req = 0;
                if (t4_persist_run) t4_marker(0);
                if (t4_persist_run) NotifyBox(4000, T4_DCOLS ? "Wide 4K %dx%d" : "Taller 4K %dx%d", (int)T4_WN, T4_H + T4_DROWS);
                else NotifyBox(10000, "Taller 4K %dx%d ON: record now (LL 10-bit). End: Debug > End taller 4K", T4_W, T4_H + T4_DROWS);
            }
            else
            {
                t4_log("revert deferred until magnify is active\n");
                if (t4_auto) NotifyBox(5000, "Taller 4K: restoring normal 4K (auto magnify)...");
                else NotifyBox(5000, "Taller 4K: magnify x5 and exit once more (restores normal 4K)");
            }
            t4_trace_ms = 3000;
            if (t4_persist_run && !hold) t4_marker(1);
            /* the hold wait has no time limit in practice (24 h); the marker goes on when the ending starts */
            rc = t4_cycle(hold ? 24 * 3600 * 1000 : 30000, &narrowed, &reflex14);
            t4_holding = 0; t4_nocancel = 0;
            if (hold) t4_log("HOLD ended (%s)\n", rc == 0 ? "magnify" : rc == 3 ? "4K path stopped" : "timeout");
            if (rc == 3) snprintf(t4_result, sizeof(t4_result), "ended (LV left 4K)");
            else if (hold && rc == 0) snprintf(t4_result, sizeof(t4_result), "ended");
            t4_log("cycle 2: %s, MEM1 %dx%d, +1c %d\n", rc == 0 ? "done" : rc == 1 ? "cancelled" : rc == 3 ? "4K path stopped" : "timeout",
                   (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14), (int)MEM(T4_PATH + 0x1c));
            if (t4_pend_all)
            {
                t4_pend_all = 0;
                t4_log("not restored (no magnify seen): power-cycle the camera\n");
                snprintf(t4_result, sizeof(t4_result), "not restored: power-cycle the camera");
            }
            patched = 0;
        }
    }
out:
    if (raw && lv && MEM(T4_STATE + 0xc) == 12 && MEM(T4_PATH + 0x1c) != T4_H && MEM(T4_PATH + 0x1c) != 1496)
    {
        /* path still tall: stop the tap first, so no 14-bit frame of the taller size reaches MEM1 */
        call("lv_save_raw", 0); raw = 0;
        t4_log("path still tall (+1c %d): tap stopped before restoring 14-bit; power-cycle advised\n", (int)MEM(T4_PATH + 0x1c));
    }
    if (packed)
    {
        /* pack back to Canon's default; width field back to what Canon set for this mode */
        uint32_t o2 = cli();
        if (MEM(T4_PATH + 0x18) == T4_WSET) MEM(T4_PATH + 0x18) = T4_W;
        MEM(T4_PATH + 0x6c) = orig_pack[2]; MEM(T4_PATH + 0x70) = orig_pack[3];
        sei(o2);
        msleep(100);
    }
    if (raw) call("lv_save_raw", 0);
    t4_log("end: %s; narrowed %d, 14-bit reflex %d; MEM1 %dx%d, +18 %d, f2 %d\n", t4_result, narrowed, reflex14,
           (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14), (int)MEM(T4_PATH + 0x18), (int)MEM(T4_PATH + 0x6c));
    if (t4_persist_run)
    {
        t4_marker(0);
        if (held) t4_c1retry = 0;
        if (!held && rc != 1 && t4_apply_fail && !t4_c1retry)
        {   /* v5.08: an occasional "cpu1 check failed" (2 of 8 Wide switches in v5.06, values already restored, still
             * in magnify): retry once automatically instead of pausing the mode */
            t4_c1retry = 1;
            t4_log("cpu1 check failed: retrying once\n");
        }
        else if (!held && rc != 1) t4_suspend = 1;   /* refused or came up wrong: no automatic retries until the mode changes */
        if (t4_key_pend) t4_log("key %x ended the taller frame\n", t4_key_pend);
        if (t4_suspend) NotifyBox(8000, "4K height paused: %s", t4_result);   /* v5.12: <= ~40 chars fit the R's screen */
    }
    else NotifyBox(8000, "Taller 4K: %s", t4_result);
    t4_running = 0;
    if (t4_key_pend)
    {
        /* v4.79: the key that ended the taller frame (MENU / PLAY / Q) goes to Canon now */
        int k = t4_key_pend; t4_key_pend = 0;
        msleep(300);
        t4_log("re-sending key %x\n", k);
        fake_simple_button(k);
    }
}


/* v4.94 read-only width census (camera v4.93: +64 cols froze ~50-100 ms after the wider rebuild): record every word in
 * low RAM 0x1000..0x2400000 (mapped: TTBR0 core L1 0-32 MB, TTBR1 supersection 0x02) whose low or high 16 bits is a
 * 4K width encoding, then magnify x5 (auto) and log the ones that changed: mode-dependent width consumers. Then
 * magnify out. Writes nothing but the log (and Canon's own magnify). */
static const uint16_t t4_cw[] = { 4256, 4255, 4254, 4252, 4250, 4248, 4112, 4104, 4100, 8508, 8510, 8512, 2128, 2127, 5320, 3040 };
static int t4_cmatch(uint32_t v)
{
    uint16_t lo = v & 0xffff, hi = v >> 16;
    for (unsigned k = 0; k < sizeof(t4_cw) / 2; k++) if (lo == t4_cw[k] || hi == t4_cw[k]) return 1;
    return 0;
}
static volatile int t4_cs_running;
static void t4_census_task(void *unused)
{
    for (int k = 0; k < 100 && gui_menu_shown(); k++) msleep(100);
    t4_log("=== width census (v4.95) at %d ms: lv %d, mode %d, MEM1 %dx%d +18 %d\n", (int)get_ms_clock(), lv,
           (int)MEM(T4_STATE + 0xc), (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14), (int)MEM(T4_PATH + 0x18));
    const int NMAX = 6000;
    uint32_t *ad = malloc(NMAX * 8);
    if (!lv || gui_menu_shown() || MEM(T4_STATE + 0xc) != 12 || lv_dispsize != 1 || !ad)
    {
        t4_log("not started (needs 4K movie LV x1, menu closed; mem %d)\n", ad != 0);
        NotifyBox(5000, "Width census: needs 4K movie LV, menu closed");
        if (ad) free(ad);
        t4_cs_running = 0; return;
    }
    uint32_t *va = ad + NMAX;   /* second half: values */
    int n = 0, over = 0;
    uint32_t t0 = get_ms_clock();
    for (uint32_t a = 0x1010; a < 0x2400000 - 0x10; a += 4)   /* neighbours a-8..a+8 stay inside the mapped range */
    {
        if ((a & 0x3ffff) == 0x10) msleep(2);   /* v4.95: yield every 256 KB (v4.94: 3.5 s busy scan, magnify never came) */
        uint32_t v = MEM(a);
        if (!t4_cmatch(v)) continue;
        if ((a >= (uint32_t)ad && a < (uint32_t)(ad + 2 * NMAX))) continue;   /* our own table */
        if (n < NMAX) { ad[n] = a; va[n] = v; n++; } else over++;
    }
    t4_log("census 4K: %d hits (%d over the cap) in %d ms\n", n, over, (int)(get_ms_clock() - t0));
    msleep(1000);
    for (int tr = 0; tr < 3 && MEM(T4_STATE + 0xc) != 13; tr++)
    {
        t4_zoom(5);   /* v4.95: retried (v4.94: the one request was ignored, mode stayed 12) */
        for (int k = 0; k < 150 && MEM(T4_STATE + 0xc) != 13; k++) msleep(20);
        t4_log("magnify request %d: mode %d, lv_dispsize %d\n", tr + 1, (int)MEM(T4_STATE + 0xc), lv_dispsize);
    }
    if (MEM(T4_STATE + 0xc) != 13)
    {
        t4_log("census: magnify not reached, no comparison\n");
        NotifyBox(6000, "Width census: magnify didn't come on (no result)");
        free(ad); t4_cs_running = 0; return;
    }
    msleep(1500);
    int m = (int)MEM(T4_STATE + 0xc), nch = 0;
    t4_log("magnify: mode %d, MEM1 %dx%d +18 %d\n", m, (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14), (int)MEM(T4_PATH + 0x18));
    for (int i = 0; i < n; i++)
    {
        uint32_t now = MEM(ad[i]);
        if (now == va[i]) continue;
        nch++;
        uint32_t a = ad[i];
        t4_log("  chg %08x: %x -> %x | %x %x [%x] %x %x\n", (int)a, (int)va[i], (int)now, (int)MEM(a - 8), (int)MEM(a - 4),
               (int)now, (int)MEM(a + 4), (int)MEM(a + 8));
    }
    t4_log("census: %d of %d changed in magnify (mode %d)\n", nch, n, m);
    t4_zoom(1);
    for (int k = 0; k < 150 && !(MEM(T4_STATE + 0xc) == 12 && lv_dispsize == 1); k++) msleep(20);
    msleep(1500);
    int back = 0;
    for (int i = 0; i < n; i++) if (MEM(ad[i]) != va[i]) back++;
    t4_log("back in 4K: mode %d, %d of %d still differ from the first 4K scan\n", (int)MEM(T4_STATE + 0xc), back, n);
    free(ad);
    NotifyBox(8000, "Census: %d changed", nch);
    t4_cs_running = 0;
}
static void t4_cselect(void *priv, int delta)
{
    if (t4_running || t4_cs_running) { NotifyBox(2000, "Taller 4K test already running"); return; }
    if (t4_pmode) { NotifyBox(3000, "Set 4K frame height to Normal first"); return; }
    t4_cs_running = 1;
    task_create("t4census", 0x1e, 0x1000, t4_census_task, 0);
    NotifyBox(4000, "Width census: close the ML menu to start");
}
static void t4_wselect(void *priv, int delta)
{
    /* v4.93: Debug > Taller 4K tests > Wider 4K, no raw tap: +64 columns, normal height, no tap (Global Draw OFF).
     * v4.96: priv 1 = "Longer line, no raw tap" (HD +16, width unchanged), 2 = wider +64 with HD +16 (blank kept) */
    int v = priv ? *(int *)priv : 2;
    if (t4_running) { NotifyBox(2000, "Taller 4K test already running"); return; }
    if (t4_pmode) { NotifyBox(3000, "Set 4K frame height to Normal first"); return; }
    t4_persist_run = 0; t4_dump = 0; t4_tap = 0; t4_hold = 0;   /* v4.98: never hold a test width */
    t4_drows = 0; t4_dcols = (v == 1 || v == 5) ? 0 : (v == 4 || v == 6) ? 56 : 64; t4_dhd = 16;
    if (v == 7) t4_dcols = 0;                            /* v5.01 W4a: longer line, normal width, 0x76 = 0xaa (left half) */
    if (v == 8) t4_dcols = 56;                           /* v5.01 W4b: v5.00 aligned +56 + 0x76 = 0x00 (full width) */
    t4_lunits = (v == 6 || v == 8) ? 6 : -1;             /* v5.00: +56 with the start on a 12-px boundary (372 * 4) */
    t4_runits = -1;
    t4_r76 = v == 7 ? 0xaa : v == 8 ? 0x00 : -1;
    if (v == 9)                                          /* v5.02 W4c: window 351..1430 (+216), MEM1 4480, HD 1500, 0x76 0 */
    { t4_dcols = 224; t4_dhd = 56; t4_lunits = 27; t4_runits = 27; t4_r76 = 0; }
    if (v >= 3) { t4_tap = 1; t4_dump = 1; }   /* v4.98: tapped +64 -> T4_4320W.BIN; v4.99: +56 -> T4_4312W.BIN */
    t4_d68 = 0; t4_d69 = 0;
    t4_running = 1;
    task_create("tall4k", 0x1d, 0x1000, t4_task, 0);
    NotifyBox(4000, v == 1 ? "Longer line: close the ML menu to start" : v == 3 ? "Wider 4K +64 (raw tap): close the ML menu" : v == 4 ? "Wider 4K +56 (raw tap): close the ML menu" : v == 5 ? "Longer line (raw tap): close the ML menu" : v == 6 ? "Wider +56, 12-px start (raw tap): close the ML menu" : v == 7 ? "Sensor 0x76 left half (raw tap): close the ML menu" : v == 8 ? "Wider +56, 0x76 full (raw tap): close the ML menu" : v == 9 ? "Wider 4K 4480 (raw tap): close the ML menu" : "Wider 4K +64: close the ML menu to start");
}
static void t4_select(void *priv, int delta)
{
    if (t4_running) { NotifyBox(2000, "Taller 4K test already running"); return; }
    if (t4_pmode) { NotifyBox(3000, "Set Taller 4K mode OFF first"); return; }
    t4_persist_run = 0; t4_dump = 1;
    t4_tap = priv ? *(int *)priv : 1;
    t4_drows = t4_steps[(t4_sel >= 0 && t4_sel <= 4) ? t4_sel : 0];
    t4_dcols = 0; t4_dhd = 0; t4_lunits = -1; t4_runits = -1; t4_r76 = -1;
    t4_d68 = 0; t4_d69 = t4_drows / 2;
    t4_running = 1;
    task_create("tall4k", 0x1d, 0x1000, t4_task, 0);
    NotifyBox(4000, "Taller 4K: close the ML menu to start");
}

static MENU_UPDATE_FUNC(t4_update)
{
    if (t4_running) MENU_SET_VALUE("RUNNING");
    else if (t4_result[0]) MENU_SET_VALUE("last: %s", t4_result);
    else MENU_SET_VALUE("OFF");
}

/* v4.77: Debug > Auto magnify test: no patches; ML requests magnify x5 and back via PROP_LV_DISPSIZE (first runtime
 * use on the R) and logs Canon's mode / MEM1 every 50 ms around each request. */
static volatile int t4_zt_running;
#if defined(R_TESTS)   /* v4.88 cleanup: Debug test tool, not built in the camera build */
static void t4_zt_watch(int ms, int want)
{
    uint32_t t0 = get_ms_clock(), tl = 0; int last = -1, lastd = -1;
    while (get_ms_clock() - t0 < (uint32_t)ms)
    {
        int m = MEM(T4_STATE + 0xc), d = lv_dispsize;
        if (m != last || d != lastd || get_ms_clock() - tl > 500)
        {
            tl = get_ms_clock();
            t4_log("  +%d ms: mode %d, lv_dispsize %d (%x/%x), MEM1 %dx%d, +18 %d\n", (int)(tl - t0), m, d, (int)t4_dz[0], (int)t4_dz[1],
                   (int)MEM(T4_PATH + 0x10), (int)MEM(T4_PATH + 0x14), (int)MEM(T4_PATH + 0x18));
            last = m; lastd = d;
        }
        if (m == want && d == (want == 13 ? 5 : 1) && get_ms_clock() - t0 > 1500) break;
        msleep(20);
    }
}
static void t4_zt_task(void *unused)
{
    for (int k = 0; k < 100 && gui_menu_shown(); k++) msleep(100);
    t4_log("=== auto magnify test (v4.78) at %d ms: lv %d, mode %d, lv_dispsize %d, prop %x/%x len %d (%d updates)\n",
           (int)get_ms_clock(), lv, (int)MEM(T4_STATE + 0xc), lv_dispsize, (int)t4_dz[0], (int)t4_dz[1], (int)t4_dz_len,
           (int)t4_dz_n);
    if (!lv || gui_menu_shown() || MEM(T4_STATE + 0xc) != 12 || lv_dispsize != 1)
    {
        t4_log("not started (needs 4K movie LV, not magnified, menu closed)\n");
        NotifyBox(5000, "Auto magnify test: needs 4K movie LV, menu closed");
        t4_zt_running = 0; return;
    }
    t4_zoom(5); t4_zt_watch(5000, 13);
    int in = MEM(T4_STATE + 0xc) == 13;
    msleep(1000);
    t4_zoom(1); t4_zt_watch(5000, 12);
    int out = MEM(T4_STATE + 0xc) == 12 && lv_dispsize == 1;
    t4_log("auto magnify test: in %s, out %s\n", in ? "OK" : "NO", out ? "OK" : "NO");
    NotifyBox(6000, "Auto magnify test: in %s, out %s", in ? "OK" : "NO", out ? "OK" : "NO");
    t4_zt_running = 0;
}
static MENU_SELECT_FUNC(t4_zt_select)
{
    if (t4_zt_running || t4_running) return;
    t4_zt_running = 1;
    task_create("t4zoom", 0x1d, 0x1000, t4_zt_task, 0);
    NotifyBox(4000, "Auto magnify test: close the ML menu to start");
}
#endif
extern int rawrec_running(void);
static MENU_SELECT_FUNC(t4_end_select)
{
    if (!t4_holding) { NotifyBox(3000, "Taller 4K is not on"); return; }
    if (rawrec_running()) { NotifyBox(3000, "Stop recording first"); return; }
    if (t4_persist_run) t4_pmode = 0;      /* v4.79: in the persistent mode End = mode OFF */
    t4_end_req = 1;
    NotifyBox(4000, "Taller 4K: close the ML menu, normal 4K follows");
}

/* v4.79: key hook (rectest_r.c handle_rawrec_keys calls this first). While the taller frame is held: MENU, PLAY
 * and Q/SET would open Canon screens where movie size / frame rate can change (a 4K rebuild at 29.97p with the
 * taller values can't fit the frame time), so they end the taller frame first and are re-sent after it. Canon's
 * own REC is blocked unless Movie > RAW video (R) is on. While recording, MENU/PLAY/Q are blocked. */
extern int rawrec_on(void);
int tall4k_keys(int k)
{
    if (gui_menu_shown()) return 1;
    if (t4_running && t4_persist_run && !t4_holding && (k == BGMT_MENU || k == BGMT_PLAY || k == BGMT_PRESS_SET || k == 0x20))
    { NotifyBox(2000, "Taller 4K: switching, one moment"); return 0; }   /* never open Canon screens mid-switch */
    if (k == 0x20 && !t4_running && t4_soon)
    { NotifyBox(2000, "Taller 4K: switching, one moment"); return 0; }   /* v4.86: REC hit the switch in v4.85 */
    if (!t4_holding) return 1;
    if (k == 0x20 && !rawrec_on()) { NotifyBox(3000, "Taller 4K: record with Movie > RAW video (R)"); return 0; }
    if (k != BGMT_MENU && k != BGMT_PLAY && k != BGMT_PRESS_SET) return 1;
    if (rawrec_running() || RECORDING) { NotifyBox(2000, "Stop recording first"); return 0; }
    if (!t4_key_pend)
    {
        t4_key_pend = k; t4_end_req = 1;   /* logged by the t4 task (no card writes from the GUI task) */
        NotifyBox(3000, "Taller 4K: back to normal 4K first...");
    }
    return 0;
}

/* v4.79 watcher: switches on when Taller 4K mode is set and 4K 23.98/24p movie LV has been idle for 2 s.
 * v4.81: a tick called every 250 ms from rectest's rr_status task instead of a task of its own: v4.79/v4.80's
 * permanent "t4watch" task gave Err 70 at every boot (KerTask.c:207 in module_task = task creation failed: the
 * DryOS task limit; heap was fine at 488 KB). The switching task exists only while a switch / hold runs. */
extern int get_global_draw(void);
extern int ml_started;
static void t4_persist_task(void *unused)
{
    t4_task(0);                             /* until the taller frame ends */
    t4_persist_run = 0;
}
void tall4k_tick(void)
{
    static int booted, last_mode, lastwhy = -1;
    static uint32_t t_start, idle_since, t_end;
    if (!ml_started) return;
    if (!t_start) { t_start = get_ms_clock(); return; }
    if (!booted)
    {
        if (get_ms_clock() - t_start < 2000) return;
        booted = 1;
        uint32_t sz = 0;
        if (FIO_GetFileSize(T4_PND, &sz) == 0)
        {
            t4_log("=== boot: crash marker found (camera stopped during a switch): Taller 4K mode was %d, now OFF\n", t4_pmode);
            FIO_RemoveFile(T4_PND);
            t4_pmode = 0;
            NotifyBox(10000, "Taller 4K mode switched OFF: the camera froze while switching last time");
        }
        last_mode = t4_pmode;
    }
    if (t4_pmode != last_mode) { last_mode = t4_pmode; t4_suspend = 0; t4_c1retry = 0; idle_since = 0; }
    t4_soon = 0;
    if (t4_running) { t_end = get_ms_clock(); idle_since = 0; return; }
    if (!t4_pmode || t4_suspend || t4_zt_running) { idle_since = 0; return; }
    if (get_ms_clock() - t_end < 1000) { idle_since = 0; t4_soon = !t4_holding; return; }
    int slot = (int)MEM(T4_SDRV + 0x70);
    int why = !lv ? 1 : MEM(T4_STATE + 0xc) != 12 ? 2 : (slot != 4 && (slot != 9 || t4_pmode >= 3)) ? 3 : CURRENT_GUI_MODE ? 100 + CURRENT_GUI_MODE
              : gui_menu_shown() ? 4 : rawrec_running() ? 5 : lv_dispsize != 1 ? 6 : get_global_draw() ? 7 : 0;
    if (why)
    {
        /* log once per value why it isn't switching (is idle LV GUI mode 0 on the R?) */
        if (why != lastwhy && why != 5) t4_log("mode watcher: waiting (%d)\n", why);   /* 5 = recording: no card writes */
        lastwhy = why;
        idle_since = 0; return;
    }
    lastwhy = 0;
    t4_soon = 1;
    if (!idle_since) { idle_since = get_ms_clock(); return; }
    if (get_ms_clock() - idle_since < 2000) return;
    idle_since = 0;
    t4_persist_run = 1; t4_dump = 0; t4_tap = 1; t4_hold = 1; t4_auto = 1; t4_place = 1; t4_dcols = 0; t4_dhd = 0; t4_lunits = -1; t4_runits = -1; t4_r76 = -1;
    t4_pmode_run = t4_pmode;
    t4_preview_run = t4_preview;
    t4_wlv_run = t4_wlv;
    t4_pk_run = t4_pk;
    t4_drows = t4_pmode == 4 ? 264 : t4_pmode == 3 ? 0 : t4_pmode == 2 ? 480 : 200;   /* v5.05: 4 = wide 16:9 */
    if (t4_pmode >= 3)
    {   /* v5.03: 4K frame height "Wide 4480x2214" = the camera-verified v5.02 set (desk W4c): sensor window 351..1430
         * (+216 px), MEM1 / receiver 4480, line HD 1500, VD 1390 (23.98p), blank 167, sensor 0x76 0 (whole row) */
        t4_dcols = 224; t4_dhd = 56; t4_lunits = 27; t4_runits = 27; t4_r76 = 0;
    }
    t4_d68 = 0; t4_d69 = t4_drows / 2;
    t4_running = 1;
    task_create("tall4k", 0x1d, 0x1000, t4_persist_task, 0);
}

/* v5.09: frame guide. Canon's live view always shows a fixed 3840x2160 window of the frame at 1:1 (valid-area cut
 * 3848x2166; its height word is ignored, lv-survey v4.91, and Canon's path has no scaling field, wider-desk D1), so in
 * the taller / wide heights the recording reaches past the screen edges. A small map in a corner of the image: yellow =
 * recorded picture (MLV active area), white = what the screen shows, with the picture size and the pixels recorded
 * beyond the screen per side. Draw-only on ML's layer; no Canon state is touched. Ticked every 250 ms from rectest's
 * rr_mark task (no task of its own: DryOS task limit). Redrawn when its corner pixel is gone (menu, NotifyBox, clear)
 * and every 4 s while idle; while recording only the pixel check runs. */
#include "bmp.h"
#include "vram.h"
CONFIG_INT("t4.guide", t4_guide, 1);    /* 0 OFF, 1 bottom left, 2 bottom right, 3 top left, 4 top right */
#define T4G_W 144
static int t4g_margin(int v, int dim, int len)
{
    int m = (v * len + dim / 2) / dim;
    if (v > 8 && m < 4) m = 4;          /* something is outside: keep a visible gap inside the 2-px border */
    return m < 2 ? 2 : m;
}
void tall4k_guide_tick(void)
{
    static int drawn, gx, gy, gw, gh, tick;
    static uint32_t sig;
    tick++;
    if (gui_menu_shown()) { drawn = 0; return; }   /* the ML menu owns the screen and clears it when it closes */
    int want = t4_guide && t4_holding && lv && lv_dispsize == 1 && DISPLAY_IS_ON && bmp_vram();
    if (!want)
    {
        if (drawn) bmp_fill(COLOR_EMPTY, gx, gy, gw, gh);
        drawn = 0;
        return;
    }
    /* recorded picture = MLV active area (rectest_r.c); screen = Canon's 3840x2160, centred like the mode places it */
    int pw = T4_DCOLS ? 4320 : 4112, ph = 2168 + (int)T4_DROWS;
    if (t4_persist_run && t4_pmode_run == 1) ph = 2312;
    if (t4_persist_run && t4_pmode_run == 4) ph = 2430;
    int sw = 3840, sh = 2160;
    int sx = (pw - sw) / 2, sy = t4_place ? (ph - sh) / 2 : 0;
    if (T4_DCOLS && t4_wlv_run == 1) sx -= 4 * (int)T4_LU;     /* Wide screen fix "Line width": view 108 px left */
    if (t4_pk_run) { sx += (sw - 3454) / 2; sy += (sh - 1943) / 2; sw = 3454; sh = 1943; }   /* v5.10 zoom proof: PREDICTED span, 3840 x 921 / 1024 */
    int l = MAX(sx, 0), r = MAX(pw - sw - sx, 0), t = MAX(sy, 0), b = MAX(ph - sh - sy, 0);
    get_yuv422_vram();                  /* refreshes os (panel / EVF image area) */
    int w = T4G_W, h = (T4G_W * ph / pw + 1) & ~1;
    int x = (t4_guide == 2 || t4_guide == 4) ? os.x_max - w - 8 : os.x0 + 8;
    int y = t4_guide >= 3 ? os.y0 + 8 : os.y_max - h - 8;
    if (x < 0 || y < 0 || x + w > 720 || y + h > 480) return;
    uint32_t s = (uint32_t)pw + (uint32_t)ph * 5000u + (uint32_t)(l * 7 + t * 13 + x * 31 + y * 101) * 25000000u;
    if (drawn && s == sig && bmp_getpixel(x, y) == COLOR_YELLOW && (rawrec_running() || tick % 16)) return;
    if (drawn && s != sig) bmp_fill(COLOR_EMPTY, gx, gy, gw, gh);
    bmp_fill(COLOR_BLACK, x, y, w, h);
    bmp_draw_rect(COLOR_YELLOW, x, y, w, h);
    bmp_draw_rect(COLOR_YELLOW, x + 1, y + 1, w - 2, h - 2);
    int ix = x + t4g_margin(l, pw, w), ix1 = x + w - t4g_margin(r, pw, w);
    int iy = y + t4g_margin(t, ph, h), iy1 = y + h - t4g_margin(b, ph, h);
    bmp_draw_rect(COLOR_WHITE, ix, iy, ix1 - ix, iy1 - iy);
    char ln[3][28];
    snprintf(ln[0], sizeof(ln[0]), "%dx%d", pw, ph);
    if (l == r) snprintf(ln[1], sizeof(ln[1]), "L/R +%d", l); else snprintf(ln[1], sizeof(ln[1]), "L+%d R+%d", l, r);
    if (t == b) snprintf(ln[2], sizeof(ln[2]), "T/B +%d", t); else snprintf(ln[2], sizeof(ln[2]), "T+%d B+%d", t, b);
    const int f = FONT(FONT_MONO_20, COLOR_WHITE, COLOR_BLACK), fh = fontspec_height(f);
    for (int k = 0; k < 3; k++)
    {
        int tw = bmp_string_width(f, ln[k]);
        if (tw > ix1 - ix - 4 || 3 * fh > iy1 - iy - 4) continue;     /* never draw over the frames */
        bmp_printf(f, (ix + ix1 - tw) / 2, (iy + iy1 - 3 * fh) / 2 + k * fh, "%s", ln[k]);
    }
    gx = x; gy = y; gw = w; gh = h; sig = s; drawn = 1;
}

extern int get_global_draw(void);
MENU_UPDATE_FUNC(tall4k_height_update)
{
    /* v4.85: Movie > RAW video (R) > 4K frame height (was Debug > Taller 4K mode, v4.79-v4.84) */
    if (!t4_pmode) return;
    if (t4_holding && t4_persist_run) MENU_SET_RINFO("ON");
    else if (t4_suspend) MENU_SET_RINFO("paused");
    else MENU_SET_RINFO("waiting");
    if (get_global_draw()) MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Needs Overlay > Global Draw OFF.");
    else if (t4_suspend && t4_nomag[0]) MENU_SET_WARNING(MENU_WARN_ADVICE, "Paused: %s. AF method 1-point AF? Then Normal and back.", t4_nomag);
    else if (t4_suspend) MENU_SET_WARNING(MENU_WARN_ADVICE, "Paused (%s): set Normal and back to retry.", t4_result[0] ? t4_result : "failed switch");
    else if (is_continuous_af()) MENU_SET_WARNING(MENU_WARN_ADVICE, "Movie Servo AF is on: Canon may refuse the zoom the switch needs. Set Disable.");
    else if (t4_pmode >= 3) MENU_SET_WARNING(MENU_WARN_INFO, "Wide: 23.98p only. Screen = middle 3840 of the 4320 px recorded (Frame guide).");
    else MENU_SET_WARNING(MENU_WARN_INFO, "Switches by itself ~2 s after idle 4K 23.98p movie LV (zooms in/out once).");
}

static int t4_one = 1, t4_zero = 0, t4_two = 2, t4_three = 3, t4_four = 4, t4_five = 5, t4_six = 6, t4_seven = 7, t4_eight = 8, t4_nine = 9;
static struct menu_entry t4_menu[] = {
    {
        .name     = "Taller 4K tests",
        .select   = menu_open_submenu,
        .help     = "Step-by-step taller 4K test tools (v4.61-v4.78). Everyday use: Movie > RAW video (R) >",
        .help2    = "4K frame height. Don't run these while a frame height other than Normal is set.",
        .children = (struct menu_entry[]) {
            {
                .name    = "Taller 4K rows",
                .priv    = &t4_sel,
                .max     = 4,
                .choices = CHOICES("+40 (2254)", "+200 (2414)", "+400 (2614)", "+480 (2694)", "+560 (2774)"),
                .help    = "Extra rows for the Taller 4K tests (v4.71). Above +40 needs Canon 4K 23.98p.",
                .help2   = "Resets to +40 at every power-on.",
            },
            {
                .name    = "Taller 4K after test",
                .priv    = &t4_hold,
                .max     = 1,
                .choices = CHOICES("Revert", "Hold for recording"),
                .help    = "Hold: keep the taller frame after the checks, record with Movie > RAW video (LL 10-bit),",
                .help2   = "then stop recording and magnify x5 + exit to restore. Resets to Revert at power-on.",
            },
#ifdef R_TESTS
            {
                .name    = "Auto magnify test",
                .select  = t4_zt_select,
                .help    = "v4.77: ML magnifies x5 and back by itself (no sensor changes). Movie 4K LV.",
                .help2   = "Result on screen and in ML/LOGS/TALL4K.TXT.",
            },
#endif
#ifdef R_TESTS
            {
                .name    = "Taller 4K magnify",
                .priv    = &t4_auto,
                .max     = 1,
                .choices = CHOICES("Manual", "Auto"),
                .help    = "Auto: the test magnifies in/out by itself (v4.77). Resets to Manual at power-on.",
            },
#endif
            {
                .name    = "End taller 4K",
                .select  = t4_end_select,
                .help    = "While held: back to normal 4K (auto magnify). Don't use while recording.",
            },
            {
                .name    = "Taller 4K placement",
                .priv    = &t4_place,
                .max     = 1,
                .choices = CHOICES("Bottom", "Centred"),
                .help    = "Bottom: extra rows below Canon's 4K window. Centred: half above, half below (v4.74).",
                .help2   = "Centred = the 4K frame height mode's placement (default since v4.88).",
            },
#ifdef R_TESTS
            {
                .name    = "Taller 4K preview",
                .priv    = &t4_preview,
                .max     = 2,
                .choices = CHOICES("Centre", "Squeeze", "Fit"),
                .help    = "DISABLED since v5.10 (always Centre): Squeeze wrote past Canon's raw buffer (desk D2),",
                .help2   = "Fit froze the camera (v4.90).",
            },
#endif
            {
                .name   = "Taller 4K (test)",
                .priv   = &t4_one,
                .select = t4_select,
                .update = t4_update,
                .help   = "Movie 4K 23.98p: sensor-window test, rows from Taller 4K rows. Follow the prompts",
                .help2  = "(magnify in/out). Reverts at the end; ML menu cancels. ML/LOGS/TALL4K.TXT, T4_<h>.BIN.",
            },
            {
                .name   = "Taller 4K, no raw tap",
                .priv   = &t4_zero,
                .select = t4_select,
                .update = t4_update,
                .help   = "A/B for the v4.65 hang: same +40-row test without ML's raw tap (MEM1 unused).",
                .help2  = "Needs Overlay > Global Draw OFF (RAW zebras keep the tap on). Movie 4K 23.98p.",
            },
            {
                .name   = "Width census",
                .select = t4_cselect,
                .update = t4_update,
                .help   = "v4.94 read-only: logs RAM words holding the 4K width that change when magnify comes on",
                .help2  = "(auto magnify in/out). Movie 4K LV, menu closed. ML/LOGS/TALL4K.TXT.",
            },
            {
                .name   = "Longer line, no raw tap",
                .priv   = &t4_one,
                .select = t4_wselect,
                .update = t4_update,
                .help   = "v4.97 A/B: line 28.88 -> 29.2 us, 1428 lines (23.98p kept), normal width, + D0510 block.",
                .help2  = "Auto magnify, ~5 s, then back. Global Draw OFF, Movie 4K 23.98p. ML/LOGS/TALL4K.TXT.",
            },
            {
                .name   = "Wider 4K, no raw tap",
                .priv   = &t4_two,
                .select = t4_wselect,
                .update = t4_update,
                .help   = "v4.97: +64 columns (4320), longer line, + D0510 block (v4.93/v4.96 missed it).",
                .help2  = "Auto magnify, ~5 s wider, then back. Global Draw OFF, Movie 4K 23.98p. ML/LOGS/TALL4K.TXT.",
            },
            {
                .name   = "Wider 4K +64, raw tap",
                .priv   = &t4_three,
                .select = t4_wselect,
                .update = t4_update,
                .help   = "v4.98: +64 columns with the longer line, ML's raw tap at 10-bit; saves rows to",
                .help2  = "ML/LOGS/T4_4320W.BIN (are the new columns real image?). Movie 4K 23.98p, Global Draw OFF.",
            },
            {
                .name   = "Wider 4K +56, raw tap",
                .priv   = &t4_four,
                .select = t4_wselect,
                .update = t4_update,
                .help   = "v4.99: +56 columns (4312): 10-bit line 5390 B = 385 whole 14-bit blocks (4320 isn't:",
                .help2  = "v4.98 pixels came out shuffled). Saves ML/LOGS/T4_4312W.BIN. Movie 4K 23.98p, Global Draw OFF.",
            },
            {
                .name   = "Longer line, raw tap",
                .priv   = &t4_five,
                .select = t4_wselect,
                .update = t4_update,
                .help   = "v5.00 control: longer line only (normal width) with the raw tap. Shuffled pixels here",
                .help2  = "= the timing causes it. Saves ML/LOGS/T4_2214L.BIN. Movie 4K 23.98p, Global Draw OFF.",
            },
            {
                .name   = "Wider +56, 12-px start, raw tap",
                .priv   = &t4_six,
                .select = t4_wselect,
                .update = t4_update,
                .help   = "v5.00: +56 with 24 px added left / 32 right, so the line starts on a 12-px boundary",
                .help2  = "like Canon's 4K. Saves ML/LOGS/T4_4312A.BIN. Movie 4K 23.98p, Global Draw OFF.",
            },
            {
                .name   = "Sensor 0x76 left half, raw tap",
                .priv   = &t4_seven,
                .select = t4_wselect,
                .update = t4_update,
                .help   = "v5.01 proof: normal width + longer line, sensor 0x76 0xff -> 0xaa (magnify-left code).",
                .help2  = "Expect image only in the left part. Saves ML/LOGS/T4_2214R.BIN. 4K 23.98p, Global Draw OFF.",
            },
            {
                .name   = "Wider +56, 0x76 full, raw tap",
                .priv   = &t4_eight,
                .select = t4_wselect,
                .update = t4_update,
                .help   = "v5.01: aligned +56 (as v5.00) with sensor 0x76 = 0x00 (full-width code, as FHD).",
                .help2  = "Expect ~4160 image cols. Saves ML/LOGS/T4_4312R.BIN. 4K 23.98p, Global Draw OFF.",
            },
            {
                .name   = "Wider 4K 4480, raw tap",
                .priv   = &t4_nine,
                .select = t4_wselect,
                .update = t4_update,
                .help   = "v5.02: window +216 (sensor 351..1430), MEM1 4480, line 30.0 us / 1390 lines (23.98p),",
                .help2  = "0x76 full. Saves ML/LOGS/T4_4480R.BIN. 4K 23.98p, Global Draw OFF, height Normal.",
            },
            {
                .name    = "Screen zoom proof",
                .priv    = &t4_pk,
                .max     = 1,
                .choices = CHOICES("OFF", "1.11x (k 103)"),
                .help    = "v5.10 test: Canon's digital-IS level 1 zoom value in the 4K post-develop table (2 words).",
                .help2   = "With a 4K frame height on: screen 1.11x closer, recording unchanged. Re-applies at menu close. Not saved.",
            },
            {
                .name    = "Wide screen fix",
                .priv    = &t4_wlv,
                .max     = 3,
                .choices = CHOICES("OFF", "Line width", "Width + centre", "Width + centre x16"),
                .help    = "v5.06: live view in the Wide heights. Line width = LV reads 4480-px lines (no bands);",
                .help2   = "Width + centre: frames like normal 4K (x 380); x16: x 384 (strip test). Re-applies at menu close.",
            },
            MENU_EOL,
        },
    },
};

static void t4_init(void *unused)
{
    menu_add("Debug", t4_menu, COUNT(t4_menu));
}

INIT_FUNC("tall4k_r", t4_init);
