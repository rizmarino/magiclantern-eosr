#ifndef _property_whitelist_h_
#define _property_whitelist_h_

#include "property.h"

// By default, we allow "reads" (property_handler, ML hooking) for any property.
//
// Initial support for Digic 6, 7, 8 cams found some prop handlers triggered
// bad behaviour (crashes, hangs).  These are blacklisted for both read and write
// until investigated and fixed.
//
// For Digic 4 and 5 cams, "writes" (prop_request_change()) are controlled
// by a global flag, CONFIG_PROP_REQUEST_CHANGE.  This is all or nothing.
//
// For D678, we disallow all "writes" via the same flag, so,
// CONFIG_PROP_REQUEST_CHANGE should be #undef for new ports in features.h.
// Writing to props can brick cams if mistakes are made.
//
// Additionally, even when defined, we now have a per property check (on D678),
// so if prop_write_allow[] is empty, no writes will be attempted even
// with CONFIG_PROP_REQUEST_CHANGE.  This allows devs to enable properties
// one at a time after checking correctness via reversing, tests, etc.

// DO NOT put the same property in both lists;
// that would allow writes but not reads, which is untested
// (and unnecessary as far as I know).

// deny reads / do not register property handlers for these
const uint32_t prop_handler_deny[] =
{
    // PROP_ISO was a defensive carry-over (sister D8 cams tag it "FIXME not a confirmed problem").
    // On the R a runtime PROP_ISO slave delivered fine + the handler decodes the byte-1 code
    // (CONFIG_PROP_ISO_BYTE1), so it is allowed for read and write -- see prop_write_allow[] below.
    PROP_MVR_REC_START, // probably related to MVR stubs being all wrong
    PROP_LV_AFFRAME // so far crash only confirmed on Digic 8
};

// allow writes / allow prop_request_change() for these:
const uint32_t prop_write_allow[] =
{
    // remote shutter (half/full press) for ML-triggered AF capture
    PROP_REMOTE_SW1,
    PROP_REMOTE_SW2,
    // shutter speed: enables expo bracketing / expo override (shutter axis).
    // R delivers PROP_SHUTTER as 2 bytes; value range coerced by prop_set_rawshutter.
    PROP_SHUTTER,
    // ISO: R delivers PROP_ISO as 4 bytes with the code in byte 1 (15=ISO100, +3/stop).
    // prop_set_rawiso converts ML APEX raw -> that code; the handler decodes it back.
    PROP_ISO,
    // aperture: R delivers PROP_APERTURE as 2 bytes with the Av code in byte 1 (9=f/2.8, +3/stop).
    // prop_set_rawaperture converts ML APEX raw -> that code; the handler decodes it back.
    PROP_APERTURE,
    PROP_PICTURE_STYLE,
    PROP_PICSTYLE_SETTINGS_STANDARD,
    PROP_PICSTYLE_SETTINGS_PORTRAIT,
    PROP_PICSTYLE_SETTINGS_LANDSCAPE,
    PROP_PICSTYLE_SETTINGS_NEUTRAL,
    PROP_PICSTYLE_SETTINGS_FAITHFUL,
    PROP_PICSTYLE_SETTINGS_MONOCHROME,
    PROP_PICSTYLE_SETTINGS_USERDEF1,
    PROP_PICSTYLE_SETTINGS_USERDEF2,
    PROP_PICSTYLE_SETTINGS_USERDEF3,
    PROP_PICSTYLE_SETTINGS_AUTO,
    PROP_PICSTYLE_SETTINGS_FINEDETAIL,
    // white balance mode + Kelvin (v4.36). v4.35 Prop watch on the R: all 4 bytes, mode codes as ML's (0 AWB,
    // 1 daylight, 3 tungsten, 8 shade, 9 Kelvin), Kelvin as the plain number; Canon sets LV and PH copies together,
    // as lens_set_kelvin() does. WB shift / custom gains are not verified and stay read-only.
    PROP_WB_MODE_LV,
    PROP_WB_KELVIN_LV,
    PROP_WB_MODE_PH,
    PROP_WB_KELVIN_PH,
    // LV lens focus step (v4.39): Canon's own FUN_e034ff36 writes this property with 1/2/3 (near) and
    // 0x8001/2/3 (far), 4 bytes - ML's lens_focus() encoding. Master FUN_e036033c forwards it to the LV AF task
    // (Tvaf DAFDeliverer message 0x38).
    PROP_LV_LENS_DRIVE_REMOTE
};

// anything not listed above will allow reads but not writes

#endif
