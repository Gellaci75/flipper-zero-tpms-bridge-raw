
# TPMS Bridge RAW — Experimental Beta v0.1

**Flipper Zero TPMS decoder with experimental RAW signal recording**

> **Development status:** Beta / Work in Progress — October 2026

This project extends the original TPMS Bridge application with experimental Sub-GHz RAW signal capture while retaining the existing TPMS decoding functionality.

## New RAW Recording Features

- **Long press LEFT:** Toggle RAW recording ON/OFF.
- **RAW: ON / RAW: OFF:** Recording mode indicator.
- **File format:** Flipper Zero `.sub` RAW files.
- **Expected storage location:** `/ext/subghz/TPMS_Bridge_RAW/`
- **Expected filenames:** `tpms_000.sub`, `tpms_001.sub`, etc.
- RAW snapshots are triggered by successfully decoded TPMS frames.

## Current Development Status

| Feature | Status |
|---|---|
| Firmware compilation | Successful |
| Application launches on Flipper Zero | Confirmed |
| RAW mode toggle | Implemented, needs further testing |
| TPMS decoding | Further testing required in modified version |
| RAW file creation | Not yet verified |
| RAW file playback | Not tested |

## Known Issues

- Long-press LEFT does not always produce consistent RAW mode feedback.
- RAW recording has not yet been confirmed by inspecting a saved `.sub` file.
- The RAW directory may not appear until a successful capture is saved.
- Generated RAW files are experimental and are not guaranteed to be replayable.

## Building

Copy the application source into:

`applications/external/tpms_bridge_raw/`

Build using:

```powershell
.\fbt.cmd fap_tpms_bridge_raw
```

Install the resulting `tpms_bridge_raw.fap` on a compatible Flipper Zero firmware.

## Credits

Based on the original [TPMS Bridge by Vishov Vladimir](https://github.com/VishovVladimir/flipper-zero-tpms).

TPMS protocol decoders originate from [rtl_433](https://github.com/merbanan/rtl_433).

Original licensing and attribution are preserved.

---

# Original TPMS Bridge Documentation

The following documentation describes the original application. In this modified version, **short LEFT** retains its original function, while **long LEFT** toggles experimental RAW recording.

---
