SOURCES = src/chip-soil-moisture-sensor.c
TARGET  = dist/chip-soil-moisture-sensor.wasm
CHIP_JSON = chip-soil-moisture-sensor.json

.PHONY: all
all: $(TARGET) dist/$(CHIP_JSON)

.PHONY: clean
clean:
	rm -rf dist

dist:
	mkdir -p dist

$(TARGET): dist $(SOURCES) src/wokwi-api.h
	clang --target=wasm32-unknown-wasi --sysroot /opt/wasi-libc \
	      -nostartfiles -Wl,--import-memory -Wl,--export-table \
	      -Wl,--no-entry -Werror -o $(TARGET) $(SOURCES)

dist/$(CHIP_JSON): dist $(CHIP_JSON)
	cp $(CHIP_JSON) dist
