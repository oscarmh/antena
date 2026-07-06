# Hardware Design — Az/El Antenna Rotator

Portable Az/El antenna rotator for 2× Yagi UHF phased array. Based on KrakenRF Discovery Drive concept, adapted for Feetech STS3250 smart servos and Raspberry Pi 4.

## Design Goals

- IP65+ weatherproof outdoor installation
- 2× Yagi UHF (435 MHz) phased antennas, ~5 kg total load
- Az: ±360° rotation | El: 0–90°
- No slip ring (±360° limit, unwind between passes)
- **Portable** — auto lat/lon/heading via GPS/IMU, no manual alignment
- Self-contained: RPi4 runs SatDump + GPredict + web UI + rotctl + servo control

---

## Mechanical BOM

| Component | Spec | Source | Notes |
|---|---|---|---|
| Turntable bearing | Stainless 304, Ø250mm | AliExpress: `304 stainless turntable bearing 250mm` | Base azimuth bearing |
| Base plate (fixed) | Aluminium 3mm, 300×300mm | AliExpress: `aluminum plate 300x300 3mm` | Bolted to mast |
| Rotating plate | Aluminium 3mm, 300×300mm | AliExpress: `aluminum plate 300x300 3mm` | Rides on turntable bearing |
| Yoke arms ×2 | Aluminium L-profile 40×40×3mm, 200mm | AliExpress: `aluminum angle 40x40 3mm` | Support elevation axis |
| Elevation shaft | Aluminium rod Ø8mm, 200mm | AliExpress: `aluminum shaft 8mm` | EL pivot |
| Flanged bearing | Ø8mm | AliExpress: `flanged bearing 8mm` | Opposite side of EL servo |
| Crossboom | Aluminium tube Ø20mm, 700mm | AliExpress: `aluminum tube 20mm 1m` | Holds both Yagis |
| U-bolt ×8 | M6, Ø20mm, stainless | AliExpress: `U bolt M6 20mm stainless` | Fix Yagi boom to crossboom |
| Tube clamp | Double saddle Ø20mm | AliExpress: `double saddle clamp 20mm` | Crossboom to EL axis |
| Electronics enclosure | IP66, aluminium, ~200×150×100mm | AliExpress: `aluminum waterproof enclosure IP66 200x150` | Houses RPi4 + URT-2 + buck |
| Cable gland ×4 | PG11, stainless | AliExpress: `PG11 cable gland stainless` | Waterproof cable entry |
| M12 connector ×2 | 4-pin, IP67 | AliExpress: `M12 connector 4pin IP67` | Servo cables |

---

## Electronics BOM

| Component | Spec | Notes |
|---|---|---|
| **Controller** | Raspberry Pi 4, 4GB | Runs Python servo control, SatDump, GPredict, web UI, rotctl |
| **Servo AZ** | Feetech STS3250, 12V, 50 kg·cm, TTL bus | Smart servo — position + speed + load feedback. ID=1 |
| **Servo EL** | Feetech STS3250, 12V, 50 kg·cm, TTL bus | Same model. ID=2 |
| **Servo interface** | Feetech URT-2 | Connected via USB-C to RPi4 → `/dev/ttyUSB0`. 12V servo power via terminal block |
| **GPS/IMU** | Witmotion WTGPS-02H | Dual antenna, MEMS IMU. Outputs heading+pitch+roll+lat/lon/alt via UART → `/dev/ttyUSB1` or GPIO UART |
| **Power monitor** | INA219, I2C addr 0x45 | Monitors 12V consumption. Connected to RPi4 I2C (GPIO2/3) |
| **Buck converter** | 12V→5V 3A (LM2596 or similar) | Powers RPi4 from 12V rail |
| **Power supply** | 12V DC, 10A | Single supply for everything |
| **RTL-SDR** | RTL-SDR Blog v4 or similar | USB to RPi4 for satellite reception via SatDump |

### Wiring

```
12V DC input
    ├── URT-2 (12V terminal) → STS3250 AZ (ID=1) + STS3250 EL (ID=2)
    ├── LM2596 buck (12V→5V/3A) → RPi4 USB-C power
    └── INA219 shunt (inline on 12V) → RPi4 I2C GPIO2/3

RPi4 USB → URT-2 USB-C     (/dev/ttyUSB0 — servo bus)
RPi4 USB → WTGPS-02H USB   (/dev/ttyUSB1 — GPS/IMU) OR GPIO14/15 UART
RPi4 USB → RTL-SDR          (/dev/bus/usb — SDR receiver)
RPi4 I2C GPIO2(SDA)/GPIO3(SCL) → INA219 (0x45)
```

### Power Budget

| Load | Peak current |
|---|---|
| STS3250 AZ (stall) | ~3A |
| STS3250 EL (stall) | ~3A |
| Raspberry Pi 4 | ~1.2A |
| RTL-SDR | ~0.3A |
| **Total with margin** | **~10A** |

---

## GPS/IMU Module

| Component | Spec | Source | Notes |
|---|---|---|---|
| **Witmotion WTGPS-02H** | GPS/GNSS + MEMS IMU | AliExpress item 1005006478238149 | Dual antenna integrated, outputs heading + pitch + roll + lat/lon/alt via UART. 0.2° heading accuracy. No external antenna needed |

**Why needed:** Portable use — auto lat/lon/alt for satellite pass prediction, auto heading for north alignment. No manual setup needed in field.

---

## Software Stack (RPi4)

| Software | Purpose |
|---|---|
| Python 3 + scservo | Servo control (STS3250 via URT-2) |
| GPredict | Satellite tracking + rotctl output |
| SatDump | Satellite signal decoding (NOAA, Meteor, GOES…) |
| rotctld (hamlib) | Rotator control daemon — GPredict → Python servo controller |
| Flask or FastAPI | Web UI for manual control |
| gpsd | GPS daemon for WTGPS-02H |

---

## Antenna Configuration

- 2× Yagi UHF 435 MHz, 9–11 elements, ~300–500g each
- Mounted phased (in-phase) on crossboom Ø20mm aluminium tube
- Crossboom length: 700mm, Yagis separated ~500mm
- Cable: phasing harness between driven elements
- Total antenna assembly weight: ~1.5–2 kg

---

## Mechanical Assembly — Key Points

1. **Turntable bearing** sits between fixed plate (bolted to mast) and rotating plate
2. **STS3250 AZ** bolted to the rotating plate; output shaft drives against fixed plate
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

---

## Power Options

### Phase 1 (current) — Mains 220V
- 12V 10A switching power supply (already ordered) → barrel jack into enclosure
- Simple, reliable for home/field use with generator

### Phase 2 (future) — Portable Solar
| Component | Spec | Notes |
|---|---|---|
| Solar panel | 50W monocrystalline 12V | AliExpress: `50W monocrystalline solar panel 12V` ~€30 |
| MPPT controller | 10A 12V | AliExpress: `MPPT solar charge controller 10A 12V` ~€10 |
| LiFePO4 battery | 12V 20Ah | Buffer for clouds/night. ~€70 |

Estimated daily balance: ~200Wh generated vs ~192Wh consumed → autonomous all day in summer.
