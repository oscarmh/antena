# Hardware Design — Az/El Antenna Rotator

Fork of [KrakenRF Discovery Drive](https://github.com/krakenrf/discovery_drive) adapted for Feetech STS3250 smart servos.

## Design Goals

- IP65+ weatherproof outdoor installation
- 2× Yagi UHF (435 MHz) phased antennas, ~5 kg total load
- Az: ±360° rotation | El: 0–90°
- No slip ring (±360° limit, unwind between passes — same as Discovery Drive)
- Self-contained: ESP32-S3 + web UI + rotctl + Stellarium support

---

## Mechanical BOM

| Component | Spec | Source | Notes |
|---|---|---|---|
| Turntable bearing | Stainless 304, Ø250mm | AliExpress: `304 stainless turntable bearing 250mm` | Base azimuth bearing |
| Base plate (fixed) | Aluminium 3mm, 300×300mm | AliExpress: `aluminum plate 300x300 3mm` | Bolted to mast |
| Rotating plate | Aluminium 3mm, 300×300mm | AliExpress: `aluminum plate 300x300 3mm` | Rides on lazy susan |
| Yoke arms ×2 | Aluminium L-profile 40×40×3mm, 200mm | AliExpress: `aluminum angle 40x40 3mm` | Support elevation axis |
| Elevation shaft | Aluminium rod Ø8mm, 200mm | AliExpress: `aluminum shaft 8mm` | EL pivot |
| Flanged bearing | Ø8mm | AliExpress: `flanged bearing 8mm` | Opposite side of EL servo |
| Crossboom | Aluminium tube Ø20mm, 700mm | AliExpress: `aluminum tube 20mm 1m` | Holds both Yagis |
| U-bolt ×8 | M6, Ø20mm, stainless | AliExpress: `U bolt M6 20mm stainless` | Fix Yagi boom to crossboom |
| Tube clamp | Double saddle Ø20mm | AliExpress: `double saddle clamp 20mm` | Crossboom to EL axis |
| Electronics enclosure | IP66, aluminium, 150×100×75mm | AliExpress: `aluminum waterproof enclosure IP66 150x100` | Houses ESP32 + URT-2 |
| Cable gland ×4 | PG11, stainless | AliExpress: `PG11 cable gland stainless` | Waterproof cable entry |
| M12 connector ×2 | 4-pin, IP67 | AliExpress: `M12 connector 4pin IP67` | Servo cables |

---

## Electronics BOM

| Component | Spec | Notes |
|---|---|---|
| **MCU** | ESP32-S3 DevKitC | Same as original Discovery Drive |
| **Servo AZ** | Feetech STS3250, 12V, 50 kg·cm, TTL bus | Smart servo — position + speed + load feedback |
| **Servo EL** | Feetech STS3250, 12V, 50 kg·cm, TTL bus | Same model, ID=2 |
| **Buck converter** | LM2596 12V→5V, 3A | Powers ESP32-S3 from 12V rail. URT-2 does NOT regulate 5V for MCU |
| **Servo interface** | Feetech URT-2 | TTL half-duplex bus converter. UART pins to ESP32 Serial2 (TX2/RX2). Servo power via 12V terminal block |
| **Power supply** | 12V DC, 10A | Single supply for everything |

> **URT-2 note:** Used in UART mode (2.54 pin header) to connect ESP32 Serial2 to the servo TTL bus. Wiring: TX→TX, RX→RX, V→V, G→G. Also useful for initial servo setup (assign IDs via USB-C + FD software). Verify if it provides regulated 5V output for ESP32 — if not, add an LM2596 buck converter (12V→5V) to the BOM.

### Wiring

```
12V DC input
    ├── URT-2 (12V terminal) → STS3250 AZ (ID=1) + STS3250 EL (ID=2)
    └── LM2596 buck (12V→5V) → ESP32-S3 5V pin

ESP32-S3 GPIO17 (TX2) → URT-2 TX
ESP32-S3 GPIO18 (RX2) → URT-2 RX
ESP32-S3 GND          → URT-2 GND
ESP32-S3 I2C SDA=GPIO7, SCL=GPIO6 → INA219 (0x45)
```

### Power Budget

| Load | Peak current |
|---|---|
| STS3250 AZ (stall) | ~3A |
| STS3250 EL (stall) | ~3A |
| ESP32-S3 + URT-2 | ~0.5A |
| **Total with margin** | **~10A** |

---

## GPS/IMU Module

| Component | Spec | Source | Notes |
|---|---|---|---|
| **Witmotion WTGPS-02H** | GPS/GNSS + MEMS IMU | AliExpress item 1005006478238149 | Dual antenna integrated, outputs heading + pitch + roll + lat/lon/alt via UART. 0.2° heading accuracy. No external antenna needed |

**Wiring:** UART (TX/RX) to ESP32 Serial1. 3.3-5V power.

---

## Antenna Configuration

- 2× Yagi UHF 435 MHz, 9–11 elements, ~300–500g each
- Mounted phased (in-phase) on crossboom Ø20mm aluminium tube
- Crossboom length: 700mm, Yagis separated ~500mm
- Cable: phasing harness between driven elements
- Total antenna assembly weight: ~1.5–2 kg

---

## Mechanical Assembly — Key Points

1. **Lazy susan** sits between fixed plate (bolted to mast) and rotating plate
2. **STS3250 AZ** is bolted to the rotating plate; its output shaft engages the fixed plate via a gear or direct drive
3. **Yoke** (two aluminium L-arms) rises from the rotating plate, ~200mm height
4. **STS3250 EL** bolted to one yoke arm; its shaft = elevation axis
5. **Flanged bearing** on opposite arm takes the other end of the EL shaft
6. **Crossboom** clamps to EL shaft via double saddle clamp
7. **Electronics box** (IP66) mounted on side of rotating plate

---

## Servo ID Assignment (do this before installation)

All STS3250 servos ship with **ID=1** by default. You must assign unique IDs before connecting them to the same bus.

**Required:** Feetech FD Debug Tool — download from [feetech.cn](https://www.feetech.cn) or search "FD Feetech Debug Tool"

**Procedure (one servo at a time):**

1. Connect **only one servo** to the URT-2 (never two with the same ID simultaneously)
2. Connect URT-2 to PC via USB-C
3. Supply 12V to the URT-2 power terminal
4. Open FD Debug Tool → select the correct COM port → click Scan
5. The servo appears with its current ID (default: 1)
6. Change ID to desired value and click Write (saves to servo EEPROM)
7. Disconnect servo, connect next one, repeat

**ID assignment:**

| Servo | ID | Axis |
|---|---|---|
| STS3250 #1 | **1** | Azimuth (AZ) |
| STS3250 #2 | **2** | Elevation (EL) |

> ⚠️ Never connect two servos with the same ID to the bus — bus collision, both will be unresponsive.

**Also verify/set while in FD tool:**
- Baud rate: **1000000 bps** (default, must match firmware)
- Operating mode: **Position mode** (default)
- Acceleration: set to **50** for gentle antenna-safe starts
