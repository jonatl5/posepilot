# IR camera options for the RoboMaster TT

> **Shelved (October 2026).** The project switched from following a person to approaching a stationary object, in daylight only, so the IR camera is no longer in the plan. Thermal cameras also can't see most objects, which sit at room temperature. This comparison is kept for reference; the payload and expansion-port limits below still apply to anything mounted on the TT.

Comparison written for the original IR camera task. Researched October 2026. Figures marked **(est.)** are not published by the vendor and need checking on a 0.1 g scale or a bench supply before buying.

## The limits that decide this

| Limit | Value | Why it matters |
| --- | --- | --- |
| Payload | **Under 20 g in practice**, and that includes the expansion kit (DJI) | The camera has to be very light |
| Expansion kit already mounted | Controller 9 g + dot-matrix / ToF module 3.5 g + extension board 1.3 g ≈ **13.8 g** | Leaves **≈ 6 g** for the camera, or **≈ 9.5 g** if the dot-matrix module is removed |
| Controller chip | ESP32-D2WD, 160 MHz, 2 MB flash, **no PSRAM** | Fine for small thermal sensors (a 160×120 frame is ~38 KB). Not suited to a normal video camera, which needs PSRAM for frame buffers |
| Expansion port | 14-pin: I2C, UART, SPI, GPIO, PWM, 5 V / 3.3 V power | Current limits on the power pins are **not published**; measure before adding anything power-hungry |
| Flight time | ~8 min (DJI) | Extra weight and power draw shorten it further |
| Center of gravity | DJI warns added payload shifts it and can cause crashes | Mount on the centerline, as low and as central as possible |

Two ways to get IR frames to the laptop:

- **Through the TT controller.** Wire the sensor to the 14-pin port and have the ESP32 forward frames over Wi-Fi. Works for low-bandwidth thermal sensors. Whether the controller can send data while the TT link is active needs testing.
- **A separate tiny Wi-Fi camera board.** It has its own ESP32 with PSRAM and streams directly; it only takes power from the TT. Needed for a normal (night-vision) camera.

## The candidates

### Thermal: sees body heat, works in full darkness

| | MLX90640 | FLIR Lepton 3.5 | Heimann HTPA80x64d | Meridian MI0802 + MI48 | Grid-EYE AMG8833 |
| --- | --- | --- | --- | --- | --- |
| Resolution | 32 × 24 | **160 × 120** | 80 × 64 | 80 × 62 | 8 × 8 |
| Frame rate | ~16 Hz in practice (Adafruit) | 8.7 Hz | 1–200 Hz | up to 25 Hz | 10 Hz |
| FOV | 55° × 35° or 110° × 75° | ~57° | depends on lens | depends on lens | 60° |
| Weight | **3.0–3.5 g** (Adafruit) to 6 g (Waveshare) | 0.9 g sensor + breakout **(est. 3–5 g)** | TO-8 can + carrier **(est.)** | module + board **(est.)** | ~2 g **(est.)** |
| Power | ~18–23 mA at 3.3 V | 150 mW (650 mW during shutter) | 25 mA at 3.3 V | ≤ 500 mW at 5 V (one module sheet) | low |
| Interface | I2C | SPI video + I2C control | SPI | SPI data + I2C control | I2C |
| Ease on TT's ESP32 | **Easy**: Arduino libraries exist | Moderate: SPI video timing is fiddly | Moderate | Moderate | Easy |
| Pose landmarks possible? | **No**: a person is a blob of a few pixels | **Maybe**: person outline visible; MediaPipe not trained on thermal | Unlikely | Unlikely | No |
| What it's good for | Cheap "where is the warm body" signal: center and size | Best night image in budget | Middle ground; hard to buy | Middle ground; power is high | Too low-res |
| Price | Cheapest | Most expensive (breakout board alone ~$66) | Distributor pricing | Distributor pricing | Cheap |

### Night vision: a normal camera without an IR filter, plus IR light

| | XIAO ESP32S3 Sense (+ NoIR lens) | AI-Thinker ESP32-CAM (+ NoIR lens) |
| --- | --- | --- |
| Resolution | OV2640 / OV3660, 2–3 MP; stream at 320–640 px wide | OV2640, 2 MP |
| Frame rate | Not published; depends on resolution and Wi-Fi **(test)** | Not published **(test)** |
| Weight | 21 × 17.8 × 15 mm; ~6 g gross listing **(est. 4–6 g)** | 27 × 40.5 mm board + 5 g camera **(est. 10 g+)** |
| Power | ESP32-S3 Wi-Fi streaming, est. 100–250 mA at 5 V **(test)** | Similar or more |
| Interface | Own Wi-Fi, 8 MB PSRAM; only needs power from the TT | Own Wi-Fi; only needs power |
| Pose landmarks possible? | **Yes, likely**: image looks like grayscale video | Yes, likely |
| Catch | Needs IR light. LEDs that light a person a few meters away draw hundreds of mA and add weight | Too heavy for the TT budget |
| Price | Cheap | Cheapest |

Ruled out for the TT:

- **USB thermal cores** (InfiRay Tiny1-C, Seek Mosaic, FLIR Boson): good images, but they need a USB host and more power. The TT has neither.
- **tCam-Mini** (Lepton 3.5 + ESP32-WROVER on one Wi-Fi board): a good design that streams Lepton frames over Wi-Fi, but its board size and weight aren't published and it likely exceeds the budget. Worth a look if the TT can't forward Lepton frames itself.

## How they compare on what PosePilot needs

| Need | MLX90640 | Lepton 3.5 | XIAO NoIR + LEDs |
| --- | :-: | :-: | :-: |
| Fits ~6 g budget | ✅ | ⚠️ depends on breakout | ⚠️ without LEDs only |
| Works in full darkness | ✅ | ✅ | ❌ needs IR light |
| Low power draw | ✅ | ✅ | ❌ especially with LEDs |
| Gives MediaPipe pose landmarks | ❌ | ⚠️ needs testing | ✅ likely |
| Easy to integrate | ✅ | ⚠️ | ✅ |
| Cost | ✅ | ❌ | ✅ |

## Recommendation

1. **Start with an MLX90640 (110° FOV, Adafruit 3 g version).** It's the only option that clearly fits the weight and power budget and works on the TT's own controller. It can't produce pose landmarks, but it can give the torso **center and apparent size** in the dark, which are exactly the main features the policy uses (`ex`, `es`). Use it as the low-light input, with MediaPipe on the built-in camera whenever there's enough light.
2. **Test the Lepton 3.5 as the upgrade.** Get its image on the bench and run MediaPipe on it before committing. If pose landmarks work on thermal, it's the best night camera that could fit. Weigh the breakout first: if it's over ~5 g it doesn't fit with the full expansion kit.
3. **Skip night vision with IR LEDs for the TT.** It works well with MediaPipe, but the LEDs' weight and power draw don't fit an aircraft with ~6 g and ~8 minutes to spare. It's a good fit for the future X500 platform.

This changes the design slightly: in the dark the state comes from a thermal blob rather than 33 landmarks. The observation spec should allow a "thermal-only" mode with center and scale but no landmark-quality features.

## Next steps

- [ ] Order an MLX90640 (110°) and wire it to the TT controller on the bench; read frames with the Arduino library
- [ ] Measure the TT's 3.3 V / 5 V pin current with the sensor attached
- [ ] Forward frames over Wi-Fi to the laptop and measure the frame rate
- [ ] Download sample Lepton / FLIR footage of people and run MediaPipe Pose on it
- [ ] Weigh the expansion kit parts and decide whether to keep the dot-matrix module
- [ ] Decide on the Lepton upgrade after the MediaPipe test

## Sources

- [DJI RoboMaster TT support and specs](https://www.dji.com/support/product/robomaster-tt) and [FAQ](https://dji.com/robomaster-tt/faq): payload, weights, flight time
- [RoboMaster TT expansion kit](https://store.dji.com/product/robomaster-tt-expansion-kit): controller MCU and 14-pin port
- [ESP32-D2WD (Grid Connect)](https://gridconnect.com/esp32-d2wd-5x5mm-wifi-bluetooth-combo-chip-2mbyte-internal-flash.html), [ESP32 forum on D2WD and PSRAM](https://esp32.com/viewtopic.php?p=7975)
- [MLX90640 datasheet (Melexis)](https://media.melexis.com/-/media/files/documents/datasheets/MLX90640-datasheet-melexis.pdf), [Adafruit MLX90640 overview](https://www.mouser.lt/pdfDocs/ProductOverview_Adafruit_MLX90640IRThermalCameraBreakoutBoards.pdf), [SparkFun MLX90640](https://sparkfun.com/products/14769), [Waveshare MLX90640](https://zbotic.in/product/waveshare-mlx90640-ir-array-thermal-imaging-camera-32x24-pixels-110-fov/)
- [FLIR Lepton 3 / 3.5 datasheet](https://www.mouser.com/datasheet/2/227/Lepton3_3_5_Data_Sheet-2580086.pdf), [Lepton Breakout Board v2](https://www.oem.flir.com/zh-cn/products/lepton-breakout-board-v2.0)
- [Heimann HTPA80x64d overview](https://azimp.ru/upload/pdf-static/Heimann%20Sensor/matrix/Overview-HTPA80x64d_Rev8.pdf)
- [Meridian MI0802 datasheet](https://files.waveshare.com/wiki/common/MI0802--Datasheet--v1.0.1.pdf), [Midas MTP-8062 module sheet](https://wmsc.lcsc.com/wmsc/upload/file/pdf/v2/lcsc/2311160141_Midas-Displays-MTP-8062_C17582838.pdf)
- [tCam-Mini (CircuitState)](https://circuitstate.com/tcminif), [tCam-Mini (CNX Software)](https://www.cnx-software.com/2021/03/02/tcam-mini-esp32-ir-thermal-camera-board/)
- [XIAO ESP32S3 Sense (Core Electronics)](https://core-electronics.com.au/seeed-studio-xiao-esp32s3-54467.html.plain.md), [Berrybase listing](https://www.berrybase.at/en/seeed-xiao-esp32s3-sense-esp32-s3r8-wlan-ble-5.0-ov2640-kamerasensor-8mb-psram-8mb-flash)
- [AI-Thinker ESP32-CAM](https://components101.com/modules/esp32-cam-camera-module), [OV2640 module weight](https://circuitmall.ng/product/ov2640-2mp-camera-module-for-esp32-cam/)
