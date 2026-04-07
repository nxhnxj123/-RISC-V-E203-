`timescale 1ns/1ps
`default_nettype none

// =========================================================================
// 顶层模块：Artix-7 RISC-V SoC + 硬件 PWM 控制器 + SBUS 支持
// =========================================================================
module artix_rv32 (
  // 差分 200MHz 时钟输入
  input  wire clk_p,
  input  wire clk_n,

  // 复位 (低电平有效)
  input  wire mcu_rst_n,

  // USB <-> UART (UART0 - 用于调试打印)
  inout  wire uart0_rx,
  inout  wire uart0_tx,
  
  // === R9DS 接收机输入 (SBUS) ===
  inout  wire r9ds_sbus_in,

  // === I2C 接口 ===  
  inout wire i2c0_scl,  
  inout wire i2c0_sda,
  inout wire i2c1_scl,  
  inout wire i2c1_sda,

  // === NRF24L01 / GPIO ===
  inout wire gpioA_4, // CSN
  inout wire gpioA_5, // MOSI
  inout wire gpioA_6, // MISO
  inout wire gpioA_7, // SCK 
  inout wire gpioA_8, // CE
  inout wire gpioA_9, // IRQ
  
  // === JTAG ===
  inout wire mcu_TCK,
  inout wire mcu_TDO,
  inout wire mcu_TMS,
  inout wire mcu_TDI,

  // === QSPI Flash ===
  inout wire qspi0_cs,
  inout wire [3:0] qspi0_dq,

  // =========================================================================
  // 电机 PWM 输出端口 (物理引脚)
  // =========================================================================
  output wire [3:0] motor_pwm
);

  // ===================================================================
  // 时钟处理 (保持不变)
  // ===================================================================
   wire clk_200m; 
   IBUFDS #( 
    .DIFF_TERM("FALSE"),        
    .IBUF_LOW_PWR("TRUE"),      
    .IOSTANDARD("DEFAULT")      
   ) IBUFDS_inst ( 
        .O(clk_200m),  
        .I(clk_p),   
        .IB(clk_n)  
   );

  wire clk_8388;
  wire CLK32768KHZ;
  wire clk_16M;     
  wire pll_locked;

  PLL sys_clk_gen (
    .reset       (~mcu_rst_n),
    .CLK_I_200M  (clk_200m),
    .CLK_O_16M   (clk_16M), 
    .CLK_O_8M388 (clk_8388),
    .locked      (pll_locked)
  );

  clkdivider rtc_clk_gen(
    .clk         (clk_8388),
    .reset       (~pll_locked),
    .clk_out     (CLK32768KHZ)
  );

  // ===================================================================
  // 信号定义
  // ===================================================================
  wire [31:0] dut_io_pads_gpioA_i_ival;
  wire [31:0] dut_io_pads_gpioA_o_oval; 
  wire [31:0] dut_io_pads_gpioA_o_oe;

  wire [31:0] dut_io_pads_gpioB_i_ival; // 输入给 SoC (SBUS 走这里)
  wire [31:0] dut_io_pads_gpioB_o_oval; // 从 SoC 输出 (PWM 走这里)
  wire [31:0] dut_io_pads_gpioB_o_oe;
  
  wire dut_io_pads_bootrom_n_i_ival;                            
  wire soc_rst_n;
  
  // PWM 信号线
  wire [3:0] motor_pwm_generated; 

  // QSPI 内部信号
  wire qspi0_sck_o_oval;
  wire qspi0_cs_0_o_oval;
  wire qspi0_dq_0_i_ival; wire qspi0_dq_0_o_oval; wire qspi0_dq_0_o_oe;
  wire qspi0_dq_1_i_ival; wire qspi0_dq_1_o_oval; wire qspi0_dq_1_o_oe;
  wire qspi0_dq_2_i_ival; wire qspi0_dq_2_o_oval; wire qspi0_dq_2_o_oe;
  wire qspi0_dq_3_i_ival; wire qspi0_dq_3_o_oval; wire qspi0_dq_3_o_oe;

  // JTAG 信号
  wire dut_io_pads_jtag_TCK_i_ival;
  wire dut_io_pads_jtag_TMS_i_ival;
  wire dut_io_pads_jtag_TDI_i_ival;
  wire dut_io_pads_jtag_TDO_o_oval;
  wire dut_io_pads_jtag_TDO_o_oe;

  // ===================================================================
  // [核心配置] 启动模式设置
  // ===================================================================
  assign dut_io_pads_bootrom_n_i_ival = 1'b0; 
  assign soc_rst_n = mcu_rst_n & pll_locked;

  // ===================================================================
  // E203 SoC 顶层实例化 (保持不变)
  // ===================================================================
  e203_soc_top dut (
    .hfextclk      (clk_16M),
    .hfxoscen      (1'b1),
    .lfextclk      (CLK32768KHZ),
    .lfxoscen      (1'b1),
    .motor_pwm_o(motor_pwm),
    // JTAG
    .io_pads_jtag_TCK_i_ival(dut_io_pads_jtag_TCK_i_ival),
    .io_pads_jtag_TMS_i_ival(dut_io_pads_jtag_TMS_i_ival),
    .io_pads_jtag_TDI_i_ival(dut_io_pads_jtag_TDI_i_ival),
    .io_pads_jtag_TDO_o_oval(dut_io_pads_jtag_TDO_o_oval),
    .io_pads_jtag_TDO_o_oe  (dut_io_pads_jtag_TDO_o_oe),

    // GPIO A
    .io_pads_gpioA_i_ival(dut_io_pads_gpioA_i_ival),
    .io_pads_gpioA_o_oval(dut_io_pads_gpioA_o_oval), 
    .io_pads_gpioA_o_oe  (dut_io_pads_gpioA_o_oe),
    
    // GPIO B
    .io_pads_gpioB_i_ival(dut_io_pads_gpioB_i_ival), // 这里连接了 SBUS
    .io_pads_gpioB_o_oval(dut_io_pads_gpioB_o_oval), // 这里连接了 PWM
    .io_pads_gpioB_o_oe  (dut_io_pads_gpioB_o_oe),

    // QSPI0
    .io_pads_qspi0_sck_o_oval   (qspi0_sck_o_oval), 
    .io_pads_qspi0_cs_0_o_oval  (qspi0_cs_0_o_oval),
    .io_pads_qspi0_dq_0_i_ival  (qspi0_dq_0_i_ival), .io_pads_qspi0_dq_0_o_oval(qspi0_dq_0_o_oval), .io_pads_qspi0_dq_0_o_oe(qspi0_dq_0_o_oe),
    .io_pads_qspi0_dq_1_i_ival  (qspi0_dq_1_i_ival), .io_pads_qspi0_dq_1_o_oval(qspi0_dq_1_o_oval), .io_pads_qspi0_dq_1_o_oe(qspi0_dq_1_o_oe),
    .io_pads_qspi0_dq_2_i_ival  (qspi0_dq_2_i_ival), .io_pads_qspi0_dq_2_o_oval(qspi0_dq_2_o_oval), .io_pads_qspi0_dq_2_o_oe(qspi0_dq_2_o_oe),
    .io_pads_qspi0_dq_3_i_ival  (qspi0_dq_3_i_ival), .io_pads_qspi0_dq_3_o_oval(qspi0_dq_3_o_oval), .io_pads_qspi0_dq_3_o_oe(qspi0_dq_3_o_oe),

    .io_pads_aon_erst_n_i_ival       (soc_rst_n),
    .io_pads_bootrom_n_i_ival        (dut_io_pads_bootrom_n_i_ival),
    .io_pads_dbgmode0_n_i_ival       (1'b1),
    .io_pads_dbgmode1_n_i_ival       (1'b1),
    .io_pads_dbgmode2_n_i_ival       (1'b1)
  );

  // QSPI Flash ... (保持原样)
  STARTUPE2 #( .PROG_USR("FALSE"), .SIM_CCLK_FREQ(0.0) ) STARTUPE2_inst (
      .CFGCLK(), .CFGMCLK(), .EOS(), .PREQ(), .CLK(1'b0), .GSR(1'b0), .GTS(1'b0), .KEYCLEARB(1'b1), .PACK(1'b0),
      .USRCCLKO(qspi0_sck_o_oval), .USRCCLKTS(1'b0), .USRDONEO(1'b1), .USRDONETS(1'b1)
  );
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("FAST")) iobuf_qspi0_cs (.O(), .IO(qspi0_cs), .I(qspi0_cs_0_o_oval), .T(1'b0));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("FAST")) iobuf_qspi0_dq0 (.O(qspi0_dq_0_i_ival), .IO(qspi0_dq[0]), .I(qspi0_dq_0_o_oval), .T(~qspi0_dq_0_o_oe));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("FAST")) iobuf_qspi0_dq1 (.O(qspi0_dq_1_i_ival), .IO(qspi0_dq[1]), .I(qspi0_dq_1_o_oval), .T(~qspi0_dq_1_o_oe));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("FAST")) iobuf_qspi0_dq2 (.O(qspi0_dq_2_i_ival), .IO(qspi0_dq[2]), .I(qspi0_dq_2_o_oval), .T(~qspi0_dq_2_o_oe));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("FAST")) iobuf_qspi0_dq3 (.O(qspi0_dq_3_i_ival), .IO(qspi0_dq[3]), .I(qspi0_dq_3_o_oval), .T(~qspi0_dq_3_o_oe));

  // I2C ... (保持原样)
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) i2c0_scl_iobuf (.O(dut_io_pads_gpioA_i_ival[18]), .IO(i2c0_scl), .I(dut_io_pads_gpioA_o_oval[18]), .T(~dut_io_pads_gpioA_o_oe[18])); PULLUP p_i2c0_scl (.O(i2c0_scl)); 
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) i2c0_sda_iobuf (.O(dut_io_pads_gpioA_i_ival[19]), .IO(i2c0_sda), .I(dut_io_pads_gpioA_o_oval[19]), .T(~dut_io_pads_gpioA_o_oe[19])); PULLUP p_i2c0_sda (.O(i2c0_sda));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) i2c1_scl_iobuf (.O(dut_io_pads_gpioA_i_ival[20]), .IO(i2c1_scl), .I(dut_io_pads_gpioA_o_oval[20]), .T(~dut_io_pads_gpioA_o_oe[20])); PULLUP p_i2c1_scl (.O(i2c1_scl)); 
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) i2c1_sda_iobuf (.O(dut_io_pads_gpioA_i_ival[21]), .IO(i2c1_sda), .I(dut_io_pads_gpioA_o_oval[21]), .T(~dut_io_pads_gpioA_o_oe[21])); PULLUP p_i2c1_sda (.O(i2c1_sda)); 

  // UART0 (Console) ... (保持原样)
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) uart0_rx_iobuf (.O(dut_io_pads_gpioA_i_ival[16]), .IO(uart0_rx), .I(dut_io_pads_gpioA_o_oval[16]), .T(~dut_io_pads_gpioA_o_oe[16]));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) uart0_tx_iobuf (.O(dut_io_pads_gpioA_i_ival[17]), .IO(uart0_tx), .I(dut_io_pads_gpioA_o_oval[17]), .T(~dut_io_pads_gpioA_o_oe[17]));

  // NRF24L01 / GPIO ... (保持原样)
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) gpioA_4_iobuf (.O(dut_io_pads_gpioA_i_ival[4]), .IO(gpioA_4), .I(dut_io_pads_gpioA_o_oval[4]), .T(~dut_io_pads_gpioA_o_oe[4])); PULLUP p_ga4 (.O(gpioA_4));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) gpioA_5_iobuf (.O(dut_io_pads_gpioA_i_ival[5]), .IO(gpioA_5), .I(dut_io_pads_gpioA_o_oval[5]), .T(~dut_io_pads_gpioA_o_oe[5])); PULLUP p_ga5 (.O(gpioA_5));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) gpioA_6_iobuf (.O(dut_io_pads_gpioA_i_ival[6]), .IO(gpioA_6), .I(dut_io_pads_gpioA_o_oval[6]), .T(~dut_io_pads_gpioA_o_oe[6])); PULLUP p_ga6 (.O(gpioA_6));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) gpioA_7_iobuf (.O(dut_io_pads_gpioA_i_ival[7]), .IO(gpioA_7), .I(dut_io_pads_gpioA_o_oval[7]), .T(~dut_io_pads_gpioA_o_oe[7])); PULLUP p_ga7 (.O(gpioA_7));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) gpioA_8_iobuf (.O(dut_io_pads_gpioA_i_ival[8]), .IO(gpioA_8), .I(dut_io_pads_gpioA_o_oval[8]), .T(~dut_io_pads_gpioA_o_oe[8])); PULLUP p_ga8 (.O(gpioA_8));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) gpioA_9_iobuf (.O(dut_io_pads_gpioA_i_ival[9]), .IO(gpioA_9), .I(dut_io_pads_gpioA_o_oval[9]), .T(~dut_io_pads_gpioA_o_oe[9])); PULLUP p_ga9 (.O(gpioA_9));

  // JTAG ... (保持原样)
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) IOBUF_jtag_TCK (.O(dut_io_pads_jtag_TCK_i_ival), .IO(mcu_TCK), .I(1'b0), .T(1'b1)); PULLUP p_tck (.O(mcu_TCK));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) IOBUF_jtag_TMS (.O(dut_io_pads_jtag_TMS_i_ival), .IO(mcu_TMS), .I(1'b0), .T(1'b1)); PULLUP p_tms (.O(mcu_TMS));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) IOBUF_jtag_TDI (.O(dut_io_pads_jtag_TDI_i_ival), .IO(mcu_TDI), .I(1'b0), .T(1'b1)); PULLUP p_tdi (.O(mcu_TDI));
  IOBUF #(.DRIVE(12), .IBUF_LOW_PWR("TRUE"), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) IOBUF_jtag_TDO (.O(), .IO(mcu_TDO), .I(dut_io_pads_jtag_TDO_o_oval), .T(~dut_io_pads_jtag_TDO_o_oe));

//  // 电机 PWM 输出 OBUF ... (保持原样)
//  OBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) obuf_pwm0 (.O(motor_pwm[0]), .I(motor_pwm_generated[0]));
//  OBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) obuf_pwm1 (.O(motor_pwm[1]), .I(motor_pwm_generated[1]));
//  OBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) obuf_pwm2 (.O(motor_pwm[2]), .I(motor_pwm_generated[2]));
//  OBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW")) obuf_pwm3 (.O(motor_pwm[3]), .I(motor_pwm_generated[3]));

endmodule
`default_nettype wire
