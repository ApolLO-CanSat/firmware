default: flash_uf2

clean:
    rm -rf build

alias config := configure
configure $PICO_BOARD="":
    mkdir -p build
    cd build && cmake ..

build PICO_BOARD="": (configure PICO_BOARD)
    cd build && make -j$(nproc)

uf2_file := "build/src/cansat_firmware.uf2"
alias flash := flash_uf2
flash_uf2 PICO_BOARD="": (build PICO_BOARD)
    python3 tools/uf2conv.py -f RP2040 -D {{ uf2_file }}
