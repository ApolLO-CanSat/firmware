target := "cansat_firmware"
build_type := "Release"
config_args := ""

build_dir := "build"

artifact := f"{{build_dir}}/src/{{target}}"

default: flash_elf

clean:
  rm -rf build

alias config := configure
configure PICO_BOARD="":
  mkdir -p {{build_dir}}
  cmake \
    -S . \
    -B {{build_dir}} \
    -G Ninja \
    -DCMAKE_BUILD_TYPE={{build_type}} \
    {{ if PICO_BOARD != "" { "-DPICO_BOARD=" + PICO_BOARD } else { "" } }} \
    {{config_args}}

build PICO_BOARD="": (configure PICO_BOARD)
  cmake --build {{build_dir}} --config {{build_type}} --target {{target}} --parallel

alias flash := flash_elf

flash_elf PICO_BOARD="": (build PICO_BOARD)
  picotool load -fvx "{{artifact}}.elf"

flash_uf2 PICO_BOARD="": (build PICO_BOARD)
  picotool reboot -fu
  python3 tools/uf2conv.py -f RP2040 -D "{{artifact}}.uf2"

serial serial_port="/dev/ttyACM0":
  picocom -b 115200 {{serial_port}}

reboot:
  picotool reboot -f