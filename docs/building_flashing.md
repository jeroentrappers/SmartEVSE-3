# Building the firmware
You can get the latest release off of https://github.com/dingo35/SmartEVSE-3.5/releases, but if you want to build it yourself:
* Install platformio-core https://docs.platformio.org/en/latest/core/installation/methods/index.html
* Clone this github project, cd to the smartevse directory where platformio.ini is located
* Compile firmware.bin: `platformio run` (or `pio run`) <br>

Following these instructions on Linux you would create firmware.bin in directory /path/to/SmartEVSE-3.5/SmartEVSE-3/.pio/build/release as follows:
```
sudo apt install platformio
git clone https://github.com/dingo35/SmartEVSE-3.5.git
cd SmartEVSE-3.5/SmartEVSE
pio run
```

## macOS (and anywhere else with `uv`)
`sudo apt install platformio` is Linux-only and often ships an outdated
version. On macOS (or any machine with [`uv`](https://docs.astral.sh/uv/)
installed) the most reliable path is:

```bash
# 1) install PlatformIO Core as a standalone, isolated tool
uv tool install platformio
pio --version   # expect 6.x

# 2) clone and build the release (v3 ESP32) env
git clone https://github.com/dingo35/SmartEVSE-3.5.git
cd SmartEVSE-3.5/SmartEVSE-3
pio run -e release
```

`pio run` needs three things, each downloaded on first use and cached
under `~/.platformio/`:

| What | When | Size |
|---|---|---|
| `espressif32` platform (~6.10.0) | first build of any ESP32 env | small |
| `framework-arduinoespressif32` (Arduino-ESP32 + ESP-IDF + xtensa GCC) | first build of `release` or `v4` | ~500 MB |
| community `platform-ch32v` | first build of `ch32` only | ~200 MB |

On a warm cache, a clean `pio run -e release` is typically under a
minute. The first ever run is 3–10 min depending on your connection.

You do **not** need to install ESP-IDF separately. PlatformIO pins and
manages the Arduino-ESP32 framework, which embeds the ESP-IDF version
it was built against. Running `idf.py` out of band is neither needed
nor recommended for this project.

### Build environments

| Env | Target | What it produces |
|---|---|---|
| `release` | SmartEVSE v3 (ESP32) | firmware for the ~3700-line `main.cpp` state machine + network stack |
| `v4` | SmartEVSE v4 (ESP32-S3) | as above, with `MODEM=1` and ISO 15118 over USB CDC |
| `ch32` | SmartEVSE v4 coprocessor (CH32V203) | RISC-V firmware for the safety-critical peripheral; separate toolchain |

```bash
pio run -e release            # v3 ESP32
pio run -e v4                 # v4 ESP32-S3
pio run -e ch32               # CH32V203 (pulls a different toolchain)
pio run -e release -t clean   # wipe one env's build
```

Artifacts land in `.pio/build/<env>/firmware.bin` (plus `.elf`,
`.map`, and partition tables).

To enable the telnet server that allows you to online view the debug logs, add the compile flag like this:
```
PLATFORMIO_BUILD_FLAGS='-DDBG=1' pio run
```

Other compile flags:
* DDBG=0 : no logging (default)
* DDBG=2 : log via USB-C connector
* DMIN_CURRENT=5 ; decrease minimum allowed current from 6A to 5A ----> THIS IS NOT FOLLOWING THE PROTOCOLS SO AT YOUR OWN RISK !!!

For versions older than v3.6.0, build the spiffs filesystem:
* Compile spiffs.bin: `pio run -t buildfs`

If you get linker errors about `mg_unpack` / `mg_unlist` (or other
mongoose `mg_…` symbols) during `pio run`, the build's pre-script
(`packfs.py`) silently failed and left `src/packed_fs.c` empty. The
root cause is nearly always that `packfs.py` shells out to `python`
(not `python3`), and your system only has `python3` on `PATH`.

On **Linux**:
```
whereis python3
cd /usr/bin
sudo ln -s /usr/bin/python3 /usr/bin/python
```

On **macOS** (Homebrew only ships `python3`, and you should not touch
`/usr/bin` under SIP), drop a user-local symlink instead — no sudo
required:
```bash
ln -s "$(which python3)" ~/.local/bin/python
python --version   # expect 3.x
```
(Ensure `~/.local/bin` is on your `PATH`. If it isn't, add
`export PATH="$HOME/.local/bin:$PATH"` to `~/.zshrc`.)

After fixing the symlink, re-run `pio run -e release`. You can verify
the fix landed by checking that `SmartEVSE-3/src/packed_fs.c` is no
longer zero bytes.

If you execute:
```
python packfs.py
```
this should generate a fresh src/packed_fs.c file.

## Vendored Cesanta Mongoose library

The embedded HTTP/WS stack comes from [cesanta/mongoose](https://github.com/cesanta/mongoose)
and is vendored as two amalgamated files:

```
SmartEVSE-3/src/mongoose.c        ← ~777 KB
SmartEVSE-3/src/mongoose.h        ← ~113 KB
SmartEVSE-3/src/MONGOOSE_VERSION  ← pin: tag, commit SHA, sha256s, pull date
```

Cesanta distributes mongoose as a drop-in amalgamation (not a library);
we follow that model rather than fighting it with a submodule. Build-time
customization happens through `-DMG_ENABLE_PACKED_FS=1` / `-DMG_TLS=…`
flags in `platformio.ini` — the sources themselves are unmodified
upstream.

### Verify the vendored copy hasn't drifted

```bash
cd SmartEVSE-3
./update_mongoose.sh         # exits 0 if sha256s match MONGOOSE_VERSION
```

Use this in CI or before shipping a release candidate. A mismatch means
someone edited `mongoose.c` or `mongoose.h` directly instead of going
through an upgrade.

### Upgrade to a newer upstream tag

```bash
cd SmartEVSE-3
./update_mongoose.sh --tag 7.18 --dry-run   # fetch + show summary
./update_mongoose.sh --tag 7.18             # actually replace files
git diff -- src/mongoose.* src/MONGOOSE_VERSION
```

The upgrade script fetches `mongoose.c` / `mongoose.h` from the upstream
git tag, compares `MG_VERSION` inside the header against the requested
tag (warns on mismatch), overwrites the two files, and regenerates
`MONGOOSE_VERSION` with the new sha256s and commit SHA. Review the diff
the same way you'd review any code change. If you need to carry local
patches, keep them in `custom_libs/patches/mongoose/*.patch` and teach
the script to apply them after the fetch — but there are none today,
and keeping it that way makes upgrades cheap.


# Flashing the firmware
1. Almost always, even when your webserver seems not to be working, the http://ipaddress/update link will be working;
   this is the simplest way to flash your firmware; with the "Choose file" option you can flash any firmware[.debug].bin you downloaded or built.
   When flashing firmware older then v3.6.0, you must also flash spiffs.bin this way.
2. Alternatively, you can connect your SmartEVSE with a USB-C cable to your computer:
   * Linux users: the device will present itself usually as /dev/ttyUSB0
   * Windows users will have to install [USB drivers](https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers)

   You can use the following flashing software:

    1. If you have installed platformio:
       ```
       pio run -t upload
       ```

       THIS IS THE PREFERRED WAY, because it also flashes your bootloader and the partitions.bin; so whatever you messed up, this will fix it!

       For versions older than v3.6.0, upload the spiffs filesystem:

       ```
       pio run -t uploadfs
       ```
    2. esptool:
       ```
       sudo apt install esptool
       esptool --port /dev/ttyUSB0 write_flash 0x10000 firmware.bin
       esptool --port /dev/ttyUSB0 write_flash 0x1c0000 firmware.bin 
       ```
    3. Flash it with a 3rd party tool:
       A nice 3rd party tool can be found here: https://github.com/marcelstoer/nodemcu-pyflasher
       Follow the instructions in the screenshot posted here: https://github.com/dingo35/SmartEVSE-3.5/issues/79
       Remember to flash to both partitions, `0x10000` and `0x1c0000` !!!

# I think I bricked my SmartEVSE
Luckily, there are no known instances of people who bricked their SmartEVSE.

Get your preferred firmware.bin from the asset zip you can download from https://github.com/dingo35/SmartEVSE-3.5/releases, and follow the
instructions in [Flashing the firmware](#flashing-the-firmware).

If all else fails, follow the [Building the Firmware](#building-the-firmware) instructions, and flash following the "pio run -t upload" path; always works!!!
