# TPMS Studio v1.11

TPMS Studio is a Flipper Zero application for receiving, decoding, saving
and replaying tyre-pressure sensor signals. It works directly on the
Flipper: no computer is required for normal reception.

The main screen can show the sensor ID, pressure, temperature, signal
strength, frame count and protocol. Repeated frames from the same sensor
are grouped together, so the display keeps the latest values instead of
adding another row for every transmission.

## Download and installation

Download the current build:

- [tpms_bridge_raw.fap](tpms_bridge_raw.fap)

Copy the FAP to the Flipper SD card, normally under:

```text
/ext/apps/Sub-GHz/
```

The build was checked with Flipper `APPCHK` against API 88.16. A firmware
with a different application API may require a rebuild from source.

Saved captures are stored in:

```text
/ext/subghz/TPMS_Bridge_RAW/
```

## Starting reception

Reception is OFF when the app opens. The default radio setting is
`AUTO433`, which alternates between FSK and OOK on 433.92 MHz.

On an empty main screen, press **OK** to start or stop reception. The
header reports the real radio state:

- `RX INT` - the Flipper's internal CC1101 is receiving
- `RX EXT` - a compatible external CC1101 is receiving
- `RX wait` - the radio is being prepared
- `RX ERR` - reception could not be started and will be retried
- `RX OFF` - reception is stopped

The app first checks for a compatible external CC1101. If it is not
available, it falls back to the internal radio. While reception is really
running, the notification LED gives a short cyan blink once per second.

## Main controls

### Main screen, before a sensor is found

- **OK** - start or stop reception
- **Up / Down** - change band and modulation
- **OK, hold** - keep the display backlight on or return to automatic light
- **Left** - switch automatic 125 kHz wake ON or OFF
- **Left, hold** - switch recognized-frame RAW saving ON or OFF
- **Right** - open the RAW file list; reception stops first
- **Back** - exit the app

### Main screen, after sensors are found

- **Up / Down** - stop reception and select another sensor
- **OK** - stop reception and open the selected sensor's details
- **Right** - stop reception and open the RAW file list

### Sensor detail screen

- **Up / Down** - select another stored sensor
- **OK or Back** - return to the sensor list

Reception stops when you leave the live main screen to browse detections,
open RAW files or transmit. This prevents the radio from being used by
two operations at once and reduces memory pressure.

## Bands and modulation

The six selectable settings are:

- `433F` - 433.92 MHz FSK
- `433O` - 433.92 MHz OOK
- `315F` - 315 MHz FSK
- `315O` - 315 MHz OOK
- `AUTO433` - alternates between 433 MHz FSK and OOK
- `AUTO315` - alternates between 315 MHz FSK and OOK

Each AUTO slot is active for four seconds. European original-equipment
sensors normally use 433.92 MHz, so `AUTO433` is the recommended starting
setting. The 315 MHz modes are mainly for North American and some Asian
sensors.

## What the screen shows

Up to eight sensors are remembered, keyed by protocol and sensor ID. Four
rows are visible at once. A row shows the latest ID, pressure,
temperature, signal level and the number of matching frames received.

The detail screen adds:

- pressure in bar, PSI and kPa
- exact RSSI and peak RSSI
- temperature when the protocol carries it
- frame count and age of the last frame
- decoded protocol and status flags
- battery state when the sensor protocol supplies it

When a ninth sensor appears, the sensor that has been silent longest is
replaced.

## Automatic wake

Some sensors sleep while the wheel is stationary. TPMS Studio can use the
Flipper's 125 kHz coil to send a wake field.

Press **Left** on the main or detail screen to select `Wake On` or
`Wake Off`. When Wake is ON and RX is running, the app sends a 700 ms wake
pulse every five seconds. The screen shows the countdown to the next
pulse. No wake pulse is sent while RX is stopped.

For a bench test, place the sensor close to the back of the Flipper, near
the 125 kHz coil. A wake signal is not required when a moving wheel makes
the sensor transmit by itself.

## RAW capture and file manager

Hold **Left** to enable `RAW On`. A file is saved only after a valid TPMS
frame has been decoded; unrelated 433 MHz remote-control traffic is not
saved merely because it is present. The same sensor is not saved again
within the next 30 seconds.

The filename format is similar to:

```text
RAW_26.10.17_1430.sub
RAW_26.10.17_1430_01.sub
```

Press **Right** to open the RAW file list. Use **Up / Down** to select a
file and **OK** to open its action menu:

- **Open** - decode the saved frame and show ID, pressure, temperature,
  RSSI and battery information when available
- **Play** - prepare Sub-GHz replay
- **Raw info** - show filename, size, frequency, protocol and TPMS metadata
- **Rename** - enter any valid filename using the Flipper text input
- **Delete** - delete the selected file after confirmation
- **Delete all** - delete all `.sub` files in the TPMS RAW folder after
  confirmation

In the Play screen, hold **OK** to repeat the transmission continuously.
Transmission stops when OK is released. The screen reports `TX INT`,
`TX EXT` or `TX OFF`, and the LED blinks magenta while transmission is
active. Replay success depends on the captured signal, receiver and local
radio regulations; use it only with equipment you own or are authorized
to test.

## Protocol support

Thirty-nine TPMS protocols are decoded. They were ported from
[rtl_433](https://github.com/merbanan/rtl_433). All compatible decoders
are tried automatically for the current modulation; no vehicle make has
to be selected manually.

FSK covers 26 protocols, including the European OEM formats. OOK covers
the other 13, mainly Schrader and aftermarket sensors.

### Hardware verification

The Renault-group decoder has been verified with real hardware. A
`407003VU0B` sensor was read on the bench and on a moving car with all
four wheels reporting. The format is associated with the Renault-Nissan
family and is reported by rtl_433 on Renault Clio, Captur and Zoe.

The other decoders were compared with rtl_433 test frames. This verifies
field positions, checksums and value scaling, but it is not the same as
testing every sensor model on a real vehicle.

### Decoded families

- **Renault, Dacia** - Renault, Renault 0435R
- **PSA: Citroen, Peugeot, Fiat, Mitsubishi** - Citroen/PSA
- **Ford** - Ford
- **Toyota, Lexus** - Toyota, PMV-107J
- **Hyundai, Kia** - Hyundai VDO, Elantra 2012, Kia
- **Honda** - Honda TRW
- **BMW, Mini** - BMW Gen2/3, BMW Gen4/5, Schrader MRXBC5A4
- **Mercedes** - Mercedes Sprinter
- **Porsche** - Porsche Boxster and Cayman
- **Audi, HUF, Beru, Continental and Sensata** - BMW Gen4/5 family
- **Fiat, Abarth, Mazda** - VDO TG1C
- **Chrysler, Jeep, Dodge** - TRW OOK and FSK
- **Opel, Saab, Vauxhall, Chevrolet, GM** - Schrader, Schrader EG53MA4,
  GM aftermarket
- **Subaru, Infiniti, Nissan** - Schrader SMD3MA4
- **Motorcycles** - Schrader motorcycle
- **Aston Martin** - SmarTire
- **Nissan** - Nissan
- **Aftermarket and trucks** - Jansite TY02S, Jansite Solar, Jansite
  TY588, Jansite TY468, iMars T240, Airpuxem, Sefis M3 and Careud,
  EezTire and TST-507, TyreGuard 400, Gear Hive, unbranded solar truck
  sensors, EGQ Q85, AVE and Steelmate

Two protocol overlaps are inherited from rtl_433. Jeep frames may match
the Citroen layout with a different pressure scale. The Schrader 3039
used by Infiniti, Nissan and Renault can be indistinguishable from the
Subaru SMD3MA4 layout. Treat unexpectedly scaled pressure readings with
caution.

## USB streaming

While the app is open, the Flipper CLI provides the `tpms_rx` command. It
prints one JSON line for each decoded frame, including the available
fields, protocol name and raw bytes. The local receiver releases the
radio for a CLI session and can take it back afterwards.

## Credits

Protocol field layouts, checksums and scales are based on
[rtl_433](https://github.com/merbanan/rtl_433). The FSK CC1101
configuration is based on [ProtoView](https://github.com/antirez/protoview).

TPMS Studio v1.11 by Laszlo.
