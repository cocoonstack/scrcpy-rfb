BUILD_DIR ?= build

.PHONY: all test docker clean

all:
	cmake -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Release
	cmake --build $(BUILD_DIR) --parallel

test: all
	$(BUILD_DIR)/scrcpy-rfb --self-test

docker:
	docker build -t scrcpy-rfb-builder -f docker/Dockerfile.builder .
	docker run --rm -v "$(CURDIR):/src" scrcpy-rfb-builder

clean:
	rm -rf $(BUILD_DIR) dist
