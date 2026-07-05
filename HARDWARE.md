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
| **Servo interface** | Feetech URT-2 | TTL half-duplex converter + powers ESP32 from 12V |
| **Power monitor** | INA219, I2C addr 0x45 | Monitors total 12V consumption (kept from original) |
| **Power supply** | 12V DC, 10A | Single supply for everything |

> The URT-2 board provides: 12V→5V/3.3V regulation for ESP32, TTL half-duplex bus conversion, and servo power distribution. No separate buck converter needed.

### Wiring

```
12V DC input
    └── URT-2
          ├── ESP32-S3 (5V regulated)
          ├── STS3250 AZ (ID=1, 12V)
          └── STS3250 EL (ID=2, 12V)

ESP32-S3 Serial2 (TX=GPIO17, RX=GPIO18) ──► URT-2 UART ──► Servo bus
ESP32-S3 I2C (SDA=GPIO7, SCL=GPIO6) ──► INA219 (0x45) [kept for power monitoring]
```

### Power Budget

| Load | Peak current |
|---|---|
| STS3250 AZ (stall) | ~3A |
| STS3250 EL (stall) | ~3A |
| ESP32-S3 + URT-2 | ~0.5A |
| **Total with margin** | **~10A** |

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
