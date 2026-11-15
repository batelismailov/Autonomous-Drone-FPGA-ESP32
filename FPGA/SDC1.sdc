# SDC1.sdc - timing constraints for drone_top (EP2C5T144C8)

# 50MHz oscillator on PIN_17 -> period 20ns
create_clock -name clk -period 20.000 [get_ports clk]
derive_clock_uncertainty

# Asynchronous UART inputs: esp_rx goes through the 2-FF synchronizer in uart_rx,
# fc_rx is only a reserved register for now. No clock-relative requirement.
set_false_path -from [get_ports {esp_rx fc_rx}]

# UART outputs and LEDs: the receiving side samples them asynchronously.
# One UART bit at 115200bps lasts about 8.68 microseconds (434 clock cycles),
# so the exact clock edge of an output change does not matter.
set_false_path -to [get_ports {esp_tx fc_tx led[*]}]
