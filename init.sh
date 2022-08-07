#!/bin/sh
rm fpga-load.dtb
mkdir /config/device-tree/overlays/fpga
dtc -I dts -O dtb -o fpga-load.dtb fpga-load.dts
cp fpga-load.dtb /config/device-tree/overlays/fpga/dtbo

rm fclk0-zynqmp.dtb
mkdir /config/device-tree/overlays/fclk0
dtc -I dts -O dtb -o fclk0-zynqmp.dtb fclk0-zynqmp.dts
cp fclk0-zynqmp.dtb /config/device-tree/overlays/fclk0/dtbo

rm axigpio-uio.dts
mkdir /config/device-tree/overlays/axigpio-uio
dtc -I dts -O dtb -o axigpio-uio.dtb axigpio-uio.dts
cp axigpio-uio.dtb /config/device-tree/overlays/axigpio-uio/dtbo