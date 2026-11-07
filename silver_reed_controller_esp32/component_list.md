# Silver Reed SK840 controller: component list

Every part on the controller circuit diagram. **Source** shows where each value comes from.

| Ref | Part | Value / rating | Role and connections | Source |
|-----|------|----------------|----------------------|--------|
| J1 | DIN plug, 8-pin male | 262° layout | To the machine's 7-pin socket. Pins 1–7 used, pin 8 unused, ground on the shield | You |
| J2 | DC power supply | +16.4 V | Solenoid power, switched to DIN pin 6 as VDRIVE. Negative to common ground | Gate circuit |
| U1 | ESP32 DevKit (classic) | 3.3 V logic, USB 5V | Controller. Onboard LED (GPIO2) and BOOT button (GPIO0) are used by the sketch | Sketch |
| U2 | OLED display, SSD1306 | 128×64, I²C 0x3C | SDA GPIO22, SCL GPIO21, VCC 3V3 | Sketch |
| Q1 | P-channel MOSFET | IRF9Z34 | High-side switch for VDRIVE | Gate circuit |
| Q2 | NPN transistor | 2N3904 | Pulls Q1's gate low when DOB is high | Gate circuit |
| R1 | Resistor | 10 kΩ | Q1 gate pull-up to the supply | Gate circuit |
| R2 | Resistor | 4.7 kΩ | Q2 base, from GPIO26 (DOB) | Gate circuit |
| R3 | Resistor, power | 10 Ω, 5 W | Peak/hold network, in parallel with C1 | Gate circuit |
| R4 | Resistor | 10 kΩ | ND1 pull-up to 3V3 (GPIO25) | Suggested |
| R5 | Resistor | 10 kΩ | CCP pull-up to 3V3 (GPIO27) | Suggested |
| R6 | Resistor | 10 kΩ | KSL pull-up to 3V3 (GPIO14) | Suggested |
| R7 | Resistor | 10 kΩ | HOK pull-up to 3V3 (GPIO13) | Suggested |
| R8 | Resistor | 100 Ω | Blue LED current limit (GPIO32) | Suggested |
| R9 | Resistor | 330 Ω | Red LED current limit (GPIO4) | Suggested |
| C1 | Electrolytic capacitor | 100 µF, 25 V | Peak/hold network. + faces the supply side | Gate circuit |
| F1 | Resettable fuse, PPTC | 0.75 A hold | Protects the VDRIVE line, between Q1 and R3/C1 | Gate circuit |
| D1 | LED, blue | — | GPIO32 through R8. Mirrors DOB, PWM dimmed | Sketch |
| D2 | LED, red | — | GPIO4 through R9. Flashes on the ! warning | Sketch |
| SW1 | Push button, normally open | — | Menu button, GPIO33 to ground, internal pull-up | Sketch |

Suggested values aren't set by the sketch, so swap in your as-built parts if they differ. Wire, headers and enclosure aren't listed.

Before connecting, confirm your knitting machine is listed in the Compatibility section of the user guide.
