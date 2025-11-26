debug:
	cargo build --features log/max_level_debug

release:
	cargo build --release --features log/max_level_error

tools/bin/brevduva_ota_upload:
	cargo install --git https://github.com/HalfVoxel/ota_flasher#1f09e2ab --features=upload --root=tools --locked

flash:
	cargo espflash flash --release --monitor --partition-table ./partitions.csv --baud 921600 --erase-parts app1 --target-app-partition app0

ota: tools/bin/brevduva_ota_upload
	cargo espflash save-image --chip esp32 --flash-size 4mb --partition-table ./partitions.csv --release target/xtensa-esp32-espidf/release/firmware.bin
	RUST_LOG=info ./tools/bin/brevduva_ota_upload upload --device "dust_collector b0:a7:32:28:92:29" --image target/xtensa-esp32-espidf/release/firmware.bin --version-file target/xtensa-esp32-espidf/release/build_id
