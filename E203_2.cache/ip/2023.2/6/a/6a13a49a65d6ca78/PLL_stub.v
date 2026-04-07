// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.2 (win64) Build 4029153 Fri Oct 13 20:14:34 MDT 2023
// Date        : Sat Jan 24 16:02:40 2026
// Host        : Simon running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ PLL_stub.v
// Design      : PLL
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7a200tfbg484-2
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix(CLK_O_8M388, CLK_O_16M, reset, locked, 
  CLK_I_200M)
/* synthesis syn_black_box black_box_pad_pin="reset,locked,CLK_I_200M" */
/* synthesis syn_force_seq_prim="CLK_O_8M388" */
/* synthesis syn_force_seq_prim="CLK_O_16M" */;
  output CLK_O_8M388 /* synthesis syn_isclock = 1 */;
  output CLK_O_16M /* synthesis syn_isclock = 1 */;
  input reset;
  output locked;
  input CLK_I_200M;
endmodule
