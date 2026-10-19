#!/bin/sh
# Icarus Verilog 12: functional test of Bootloader 3.3
cd "$(dirname "$0")"
python3 gen_stim.py
iverilog -g2012 -o tb.vvp tb.v models.v epcs_model.v ../Bootloader.v ../net_rx.v ../net_tx.v ../net_proc.v \
  ../page_ram.v ../Flash.v ../crc32_d8.v ../cdc_sync.v ../pic_control.v ../uart_rx.v ../uart_tx.v \
  ../phy_cfg.v ../MDIO.v ../Reconfigure.v ../Led_flash.v ../Led_control.v
vvp -n tb.vvp > sim.log
python3 check.py
python3 test_pytool.py
