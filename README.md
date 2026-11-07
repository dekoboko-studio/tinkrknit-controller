# TinkrkniT Controller

Firmware for a USB controller that lets [TinkrkniT](https://tinkrknit.com)
knit patterns on a Silver Reed or Knitmaster electronic flatbed knitting
machine. It runs on a classic ESP32 DevKit, plugs into the machine's DIN
socket, and selects needles row by row as the carriage passes. Only the
Silver Reed SK840 has been tested so far.

**The user guide is at <https://tinkrknit.com/guide/>.** It covers which
machines are compatible, building and wiring the controller, uploading this
firmware, and knitting with it. Read the compatibility section before
connecting anything to your machine.

## What's here

| File | What it is |
|---|---|
| [silver_reed_controller_esp32.ino](silver_reed_controller_esp32/silver_reed_controller_esp32.ino) | The firmware sketch. The pin map is at the top. |
| [component_list.md](silver_reed_controller_esp32/component_list.md) | Every part on the controller circuit, with values. |

## Uploading the firmware

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) and the
   ESP32 board package (*Boards Manager*, search "esp32" by Espressif).
2. Install the **Adafruit SSD1306** and **Adafruit GFX** libraries
   (*Library Manager*).
3. Open `silver_reed_controller_esp32/silver_reed_controller_esp32.ino`,
   choose **ESP32 Dev Module** as the board and the controller's port, and
   upload.

Then open TinkrkniT in Chrome or Edge and connect from the Knit screen. The
guide has the details.

> **ESP32 pins are not 5V tolerant.** Pull the carriage signal lines up to
> 3.3V, never 5V, as the component list shows.

## License

MIT, see [LICENSE](LICENSE). The TinkrkniT name and logo are not covered by
it: a modified version should have its own. Credits are in [NOTICE](NOTICE).
This firmware is based on
[silver-reed-write](https://github.com/knitscape/silver-reed-write) by
knitscape.
