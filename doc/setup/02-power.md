# 2. Power Topology

Everything on the rover runs from a single 2s2p pack of 18650 cells through one DC-DC step-down. Motors bypass the DC-DC and pull direct from the pack. A single USB adapter into the Pi's micro-USB port is enough to bench-test the whole stack *except the motors*.

---

## 2.1 Battery pack

4× 18650 Li-ion cells wired **2s2p** (two in series → two parallel strings of two):

- Nominal: **7.4 V**
- Fully charged: **8.4 V**
- Discharge cutoff: **6.0 V** (3.0 V / cell — use a BMS to enforce)

Use a matched BMS for 2s packs with balance leads; the XL4015 and TB6612 both tolerate the full voltage range.

---

## 2.2 Rails

```
  ┌── 2s2p 18650 pack (6.0 – 8.4 V) ────────────────────┐
  │                                                     │
  ├──▶ TB6612 VM ──▶ MotorA / MotorB (direct switching) │
  │                                                     │
  └──▶ XL4015 DC-DC step-down (set to 5.1 V)            │
            │                                            │
            ├──▶ Pi 5 V rail (header pin 4)             │
            │        │                                   │
            │        ├──▶ GPS module VCC (header pin 2) │
            │        ├──▶ Pi 3V3 regulator               │
            │        │       └──▶ TB6612 VCC (pin 1)    │
            │        │       └──▶ Pi internal peripherals│
            │        └──▶ MIPI camera (CSI connector)   │
            │                                            │
            └──▶ Modem VBUS (soldered to USB test pad)  │

  GND common across battery, DC-DC, Pi, modem, GPS, TB6612
```

The XL4015 feeds **all** low-voltage loads. Nothing runs off the Pi's own 5 V regulator capacity — the 5.1 V on the Pi's header pin 2/4 is actually the DC-DC output backfed through the GPIO header (pin 4). The GPS and modem just tap the same rail at different physical locations (pin 2 and a soldered test pad, respectively).

---

## 2.3 Component power summary

| Component | Voltage | Source | Connection |
|---|---|---|---|
| Pi Zero 2W | 5.1 V | XL4015 | Header pins **4 (5 V)** + **6 (GND)** |
| GPS (NEO-6M) | 5.1 V | Pi 5 V rail (same XL4015) | Header pins **2 (5 V)** + **9 (GND)** |
| TB6612 VCC (logic) | 3.3 V | Pi 3V3 | Header pin **1 (3V3)** + GND |
| TB6612 VM (motor) | 7.4 V pack | Battery direct | Via battery → VM input |
| Motors (A + B) | switched VM | TB6612 AO / BO | Via TB6612 outputs |
| UZ801 modem | 5.1 V | XL4015 *directly* | Soldered to USB-port test pad on Pi underside |
| MIPI camera | 3.3 V | Pi CSI | Ribbon cable |

---

## 2.4 Why the modem does NOT power off a Pi pin

The UZ801 pulls **1–2 A peaks** during LTE TX. The Pi Zero 2W's 3V3 rail is limited (~250 mA) and the 5 V rail — even back-fed via pin 4 — routes through thin PCB traces that sag under modem peaks and reset the Pi.

The custom modem cable therefore:

- Takes **VBUS (5 V)** directly from the XL4015 output node (same node that feeds the Pi's pin 4), *not* from the Pi's USB connector.
- Takes **D+ / D−** from the USB data-port test pads on the **bottom side of the Pi** (near the USB micro-B connector).
- Shares GND with the rest of the system.

Cable geometry: USB-A male at the modem end, 4 flying leads at the Pi end (VBUS to XL4015, D+ / D− soldered to test pads, GND to star ground).

See [03-wiring.md §3.5](03-wiring.md#35-uz801-lte-modem--custom-cable-required) for test-pad locations and soldering notes.

---

## 2.5 Bench-debug mode (no motors)

For desk debugging — everything runs off a single 5 V USB supply into the Pi's micro-USB **PWR** port:

- Pi boots from micro-USB 5 V.
- Pi back-feeds 5 V to its own header pin 2 / 4 (the GPIO header is electrically the same rail as the micro-USB input).
- GPS (pin 2), TB6612 logic (pin 1, 3V3), modem (via the soldered test-pad wire — VBUS wire shorted to the Pi's 5 V rail at that node instead of to the XL4015) all power up normally.
- **Motors stay off** because VM is not connected — they only come alive when the battery pack is plugged in.

This lets you validate MAVLink, video, modem, GPS, and parameter handling on your desk without a fire risk from uncommanded motor movement.

---

## 2.6 Wiring recommendations

- **Star ground** at the DC-DC output. Every GND wire (Pi, modem, TB6612, GPS) lands on the same point to avoid ground loops that corrupt I2C and UART.
- **Fuse the battery output** (5–10 A) before the XL4015 and before the TB6612 VM — a stuck motor or shorted cable would otherwise try to dump the full 2s2p discharge current.
- **Decoupling caps:** add 100 µF across VM at the TB6612 and across VBUS at the modem's VBUS tap to keep the Pi from browning out on peaks.
- **Do not** power the Pi from both the micro-USB PWR port *and* the GPIO header at the same time — pick one for a given session.

---

## Next

→ [03-wiring.md](03-wiring.md) — signal wiring for motors, GPS, camera, gimbal, and the custom modem cable.
