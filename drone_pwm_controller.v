`timescale 1ns/1ps

module drone_pwm_controller (
    input  wire        clk,       // 系统时钟 (16MHz)
    input  wire        rst_n,     // 复位信号

    // ICB 总线命令通道
    input  wire        i_icb_cmd_valid, 
    output wire        i_icb_cmd_ready,
    input  wire [31:0] i_icb_cmd_addr, 
    input  wire        i_icb_cmd_read, 
    input  wire [31:0] i_icb_cmd_wdata,
    input  wire [3:0]  i_icb_cmd_wmask, 

    // ICB 总线反馈通道
    output wire        i_icb_rsp_valid,
    input  wire        i_icb_rsp_ready,
    output wire        i_icb_rsp_err,
    output wire [31:0] i_icb_rsp_rdata,

    // PWM 输出到电机
    output reg  [3:0]  motor_pwm
);

    // -------------------------------
    // 参数定义 (假设 16MHz 时钟)
    // -------------------------------
    parameter TICKS_PER_US = 16;
    parameter PWM_PERIOD   = 20000 * TICKS_PER_US; // 20ms (50Hz)

    // -------------------------------
    // 寄存器定义
    // -------------------------------
    reg [1:0]  reg_mode;       
    reg [13:0] reg_pwm_width0; 
    reg [13:0] reg_pwm_width1; 
    reg [13:0] reg_pwm_width2; 
    reg [13:0] reg_pwm_width3; 

    // -------------------------------
    // ICB 总线握手逻辑
    // -------------------------------
    assign i_icb_cmd_ready = 1'b1; 
    
    reg rsp_valid_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rsp_valid_reg <= 1'b0;
        else        rsp_valid_reg <= i_icb_cmd_valid && i_icb_cmd_ready;
    end
    
    assign i_icb_rsp_valid = rsp_valid_reg;
    assign i_icb_rsp_err   = 1'b0;  
    
    wire [11:0] addr_offset = i_icb_cmd_addr[11:0];
    assign i_icb_rsp_rdata = (addr_offset == 12'h000) ? {30'b0, reg_mode} :
                             (addr_offset == 12'h004) ? {2'b0, reg_pwm_width0, 2'b0, reg_pwm_width1} :
                             (addr_offset == 12'h008) ? {2'b0, reg_pwm_width2, 2'b0, reg_pwm_width3} : 32'b0;

    // -------------------------------
    // 寄存器写逻辑
    // -------------------------------
    wire icb_write_en = i_icb_cmd_valid && i_icb_cmd_ready && !i_icb_cmd_read;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_mode <= 2'b00;
            reg_pwm_width0 <= 14'd0;
            reg_pwm_width1 <= 14'd0;
            reg_pwm_width2 <= 14'd0;
            reg_pwm_width3 <= 14'd0;
        end else if (icb_write_en) begin
            case (addr_offset)
                12'h000: reg_mode <= i_icb_cmd_wdata[1:0];
                12'h004: begin
                    reg_pwm_width0 <= i_icb_cmd_wdata[29:16];
                    reg_pwm_width1 <= i_icb_cmd_wdata[13:0];
                end
                12'h008: begin
                    reg_pwm_width2 <= i_icb_cmd_wdata[29:16];
                    reg_pwm_width3 <= i_icb_cmd_wdata[13:0];
                end
                default: ; 
            endcase
        end
    end

    // -------------------------------
    // PWM 产生逻辑
    // -------------------------------
    reg [31:0] cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cnt <= 32'd0;
        else if (cnt >= PWM_PERIOD - 1) cnt <= 32'd0;
        else cnt <= cnt + 1;
    end

    reg [31:0] high_ticks [3:0];
    wire [13:0] widths [3:0];
    assign widths[0] = reg_pwm_width0; assign widths[1] = reg_pwm_width1;
    assign widths[2] = reg_pwm_width2; assign widths[3] = reg_pwm_width3;

    always @(*) begin
        for (integer i=0; i<4; i=i+1) begin
            case (reg_mode)
                2'b00: high_ticks[i] = 32'd0; 
                
                // 【修改点 1】MAX 模式：修改为 2200us (对应 11% 占空比)
                2'b01: high_ticks[i] = 2200 * TICKS_PER_US; 
                
                // 【修改点 2】MIN 模式：修改为 1200us (对应 6% 占空比)
                2'b10: high_ticks[i] = 1000 * TICKS_PER_US; 
                
                // 【修改点 3】RUN 模式：基础偏移量修改为 1200us。
                // 此时若 C 代码输入 widths 为 0~1000，则输出范围就是 1200us ~ 2200us。
                2'b11: high_ticks[i] = (1000 + {18'b0, widths[i]}) * TICKS_PER_US; 
                
                // 默认安全状态修改为 1200us
                default: high_ticks[i] = 1000 * TICKS_PER_US;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            motor_pwm <= 4'b0000;
        end else begin
            for (integer i=0; i<4; i=i+1) begin
                motor_pwm[i] <= (high_ticks[i] > 0) && (cnt < high_ticks[i]);
            end
        end
    end

endmodule
