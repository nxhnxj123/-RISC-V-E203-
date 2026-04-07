###############################
# Global Voltage Configuration
###############################
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS true [current_design] 
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design] 
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design] 
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE Yes [current_design]
###############################
# 200MHz Differential Clock
# From manual: R4/T4 锟?? 200M LVDS clock
###############################
set_property -dict {PACKAGE_PIN R4 IOSTANDARD DIFF_SSTL15} [get_ports clk_p]

###################################
# UART (From CH340 锟?? BANK14)
# TX: P15  RX: P14
###################################
set_property PACKAGE_PIN P15 [get_ports uart0_tx]
set_property PACKAGE_PIN P14 [get_ports uart0_rx]
set_property IOSTANDARD LVCMOS33 [get_ports {uart0_tx uart0_rx}]
##################################
#I2C
##################################
# SCL -> 物理管脚 C18 
set_property PACKAGE_PIN C18 [get_ports i2c0_scl]
set_property IOSTANDARD LVCMOS33 [get_ports i2c0_scl]
# 如果板子上没有外部上拉电阻，建议在 XDC 中也开启内部上拉（虽然 Verilog 里写了 PULLUP，这里双重保险）
set_property PULLUP true [get_ports i2c0_scl]
# SDA -> 物理管脚 F18 
set_property PACKAGE_PIN F18 [get_ports i2c0_sda]
set_property IOSTANDARD LVCMOS33 [get_ports i2c0_sda]
set_property PULLUP true [get_ports i2c0_sda]

# SCL -> 物理管脚 C19 
set_property PACKAGE_PIN C19 [get_ports i2c1_scl]
set_property IOSTANDARD LVCMOS33 [get_ports i2c1_scl]
# 如果板子上没有外部上拉电阻，建议在 XDC 中也开启内部上拉（虽然 Verilog 里写了 PULLUP，这里双重保险）
set_property PULLUP true [get_ports i2c1_scl]
# SDA -> 物理管脚 E18 
set_property PACKAGE_PIN E18 [get_ports i2c1_sda]
set_property IOSTANDARD LVCMOS33 [get_ports i2c1_sda]
set_property PULLUP true [get_ports i2c1_sda]

###################################
# Reset button (manual: R14)
###################################
set_property PACKAGE_PIN R14 [get_ports mcu_rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports mcu_rst_n]

#=========================== Debug JTAG ======================================

set_property CLOCK_DEDICATED_ROUTE FALSE [get_ports mcu_TCK]

# mcu_TCK
# mcu_TDO
# mcu_TMS
# rst
# mcu_TDI
# JTAG Debug Port
set_property -dict { PACKAGE_PIN E19  IOSTANDARD LVCMOS33 } [get_ports { mcu_TCK }]; #V20
set_property -dict { PACKAGE_PIN B20  IOSTANDARD LVCMOS33 } [get_ports { mcu_TDO }]; #U20
set_property -dict { PACKAGE_PIN D19  IOSTANDARD LVCMOS33 } [get_ports { mcu_TMS }]; #T20
set_property -dict { PACKAGE_PIN A20  IOSTANDARD LVCMOS33 } [get_ports { mcu_TDI }]; #P20


#------------------------ End of Debug JTAG ----------------------------------

set_property PACKAGE_PIN D17 [get_ports {motor_pwm[0]}]
set_property PACKAGE_PIN F16 [get_ports {motor_pwm[1]}]
set_property PACKAGE_PIN C17 [get_ports {motor_pwm[2]}]
set_property PACKAGE_PIN E17 [get_ports {motor_pwm[3]}]

set_property IOSTANDARD LVCMOS33 [get_ports {motor_pwm[*]}]

# ==============================================================================
# Flash Boot 必须的配置
# ==============================================================================
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

# ==============================================================================
# Flash 引脚约束 (CS, DQ0-DQ3)
# 请务必对照原理图修改 PIN_xx
# SCK 引脚不需要约束 (由 STARTUPE2 自动管理)
# ==============================================================================
set_property PACKAGE_PIN T19 [get_ports qspi0_cs]
set_property IOSTANDARD LVCMOS33 [get_ports qspi0_cs]

set_property PACKAGE_PIN P22 [get_ports {qspi0_dq[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {qspi0_dq[0]}]

set_property PACKAGE_PIN R22 [get_ports {qspi0_dq[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {qspi0_dq[1]}]

set_property PACKAGE_PIN P21 [get_ports {qspi0_dq[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {qspi0_dq[2]}]

set_property PACKAGE_PIN R21 [get_ports {qspi0_dq[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {qspi0_dq[3]}]

###################################
# ===== JM1 40P (BANK16, 3.3V) =====
# Mapping strictly from manual
###################################

#set_property -dict {PACKAGE_PIN F18 IOSTANDARD LVCMOS33 } [get_ports {gpioA_25 }]
#set_property -dict {PACKAGE_PIN C18 IOSTANDARD LVCMOS33 } [get_ports {gpioA_24 }]
#set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS33 } [get_ports {gpioA_23 }]
#set_property -dict {PACKAGE_PIN C17 IOSTANDARD LVCMOS33 } [get_ports {gpioA_22 }]
#set_property -dict {PACKAGE_PIN F16 IOSTANDARD LVCMOS33 } [get_ports {gpioA_21 }]
#set_property -dict {PACKAGE_PIN D17 IOSTANDARD LVCMOS33 } [get_ports {gpioA_20 }]

# IRQ (GPIO 7)
set_property -dict {PACKAGE_PIN B13 IOSTANDARD LVCMOS33 } [get_ports {gpioA_9}]
# CE (GPIO 6)
set_property -dict {PACKAGE_PIN D14 IOSTANDARD LVCMOS33 } [get_ports {gpioA_8}]
# SCK (GPIO 5)
set_property -dict {PACKAGE_PIN C13 IOSTANDARD LVCMOS33 } [get_ports {gpioA_7}]
# MISO (GPIO 4)
set_property -dict {PACKAGE_PIN F14 IOSTANDARD LVCMOS33 } [get_ports {gpioA_6}]
# MOSI (GPIO 3)
set_property -dict {PACKAGE_PIN D16 IOSTANDARD LVCMOS33 } [get_ports {gpioA_5}]
# CSN (GPIO 2)
set_property -dict {PACKAGE_PIN E16 IOSTANDARD LVCMOS33 } [get_ports {gpioA_4}]

set_property PACKAGE_PIN F13 [get_ports r9ds_sbus_in]
set_property IOSTANDARD LVCMOS33 [get_ports r9ds_sbus_in]

# 这一步不是必须的，但对于输入引脚加个上拉通常更安全，防止悬空乱跳
set_property PULLUP true [get_ports r9ds_sbus_in]
# ###################################
# # ===== JM2 40P (BANK15, 3.3V) =====
# # Mapping from manual page 16
# ###################################
# set_property PACKAGE_PIN G17 [get_ports {jm2_io[5]}]
# set_property PACKAGE_PIN N22 [get_ports {jm2_io[6]}]
# set_property PACKAGE_PIN G18 [get_ports {jm2_io[7]}]
# set_property PACKAGE_PIN M22 [get_ports {jm2_io[8]}]
# set_property PACKAGE_PIN G15 [get_ports {jm2_io[9]}]
# set_property PACKAGE_PIN H17 [get_ports {jm2_io[10]}]
# set_property PACKAGE_PIN G16 [get_ports {jm2_io[11]}]
# set_property PACKAGE_PIN H18 [get_ports {jm2_io[12]}]
# set_property PACKAGE_PIN J14 [get_ports {jm2_io[13]}]
# set_property PACKAGE_PIN J15 [get_ports {jm2_io[14]}]
# set_property PACKAGE_PIN H14 [get_ports {jm2_io[15]}]
# set_property PACKAGE_PIN H15 [get_ports {jm2_io[16]}]
# set_property PACKAGE_PIN H13 [get_ports {jm2_io[17]}]
# set_property PACKAGE_PIN M21 [get_ports {jm2_io[18]}]
# set_property PACKAGE_PIN G13 [get_ports {jm2_io[19]}]
# set_property PACKAGE_PIN L21 [get_ports {jm2_io[20]}]
# set_property PACKAGE_PIN J20 [get_ports {jm2_io[21]}]
# set_property PACKAGE_PIN H20 [get_ports {jm2_io[22]}]
# set_property PACKAGE_PIN J21 [get_ports {jm2_io[23]}]
# set_property PACKAGE_PIN G20 [get_ports {jm2_io[24]}]
# set_property PACKAGE_PIN J19 [get_ports {jm2_io[25]}]
# set_property PACKAGE_PIN K21 [get_ports {jm2_io[26]}]
# set_property PACKAGE_PIN H19 [get_ports {jm2_io[27]}]
# set_property PACKAGE_PIN K22 [get_ports {jm2_io[28]}]
# set_property PACKAGE_PIN K18 [get_ports {jm2_io[29]}]
# set_property PACKAGE_PIN J22 [get_ports {jm2_io[30]}]
# set_property PACKAGE_PIN K19 [get_ports {jm2_io[31]}]
# set_property PACKAGE_PIN H22 [get_ports {jm2_io[32]}]
# set_property PACKAGE_PIN L19 [get_ports {jm2_io[37]}]
# set_property PACKAGE_PIN M18 [get_ports {jm2_io[38]}]
# set_property PACKAGE_PIN L20 [get_ports {jm2_io[39]}]
# set_property PACKAGE_PIN L18 [get_ports {jm2_io[40]}]

# # IOSTANDARD
# foreach i [list \
#  5 6 7 8 9 10 11 12 13 14 15 16 \
#  17 18 19 20 21 22 23 24 25 26 27 28 \
#  29 30 31 32 37 38 39 40] {
#     set_property IOSTANDARD LVCMOS33 [get_ports "jm2_io[$i]"]
# }
