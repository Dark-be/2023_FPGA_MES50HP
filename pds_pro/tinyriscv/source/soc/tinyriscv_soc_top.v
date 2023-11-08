 /*                                                                      
 Copyright 2020 Blue Liang, liangkangnan@163.com
                                                                         
 Licensed under the Apache License, Version 2.0 (the "License");         
 you may not use this file except in compliance with the License.        
 You may obtain a copy of the License at                                 
                                                                         
     http://www.apache.org/licenses/LICENSE-2.0                          
                                                                         
 Unless required by applicable law or agreed to in writing, software    
 distributed under the License is distributed on an "AS IS" BASIS,       
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and     
 limitations under the License.                                          
 */

`include "../core/defines.v"
// Modified by Darkbe on 2023-11-03: remove jtag and add num of gpio
// and add an signal to avoid errors caused by "over_r" "succ_r"
// tinyriscv soc顶层模块
module tinyriscv_soc_top(
    
    
    input wire sys_clk,
    input wire rst,

    output reg over_r,         // 测试是否完成信号
    output reg succ_r,         // 测试是否成功信号

    output wire halted_ind,  //

    input wire uart_debug_pin, // 串口下载使能引脚

    output wire uart_tx_pin, // UART发送引脚
    input wire uart_rx_pin,  // UART接收引脚
    inout wire[15:0] gpio,    // GPIO引脚

    input wire spi_miso,     // SPI MISO引脚
    output wire spi_mosi,    // SPI MOSI引脚
    output wire spi_ss,      // SPI SS引脚
    output wire spi_clk      // SPI CLK引脚
    
    
    );

    wire clk;
    // master 0 interface
    wire[`MemAddrBus] m0_addr_i;
    wire[`MemBus] m0_data_i;
    wire[`MemBus] m0_data_o;
    wire m0_ack_o;
    wire m0_req_i;
    wire m0_we_i;

    // master 1 interface
    wire[`MemAddrBus] m1_addr_i;
    wire[`MemBus] m1_data_i;
    wire[`MemBus] m1_data_o;
    wire m1_ack_o;
    wire m1_req_i;
    wire m1_we_i;

    // master 2 interface
    wire[`MemAddrBus] m2_addr_i;
    wire[`MemBus] m2_data_i;
    wire[`MemBus] m2_data_o;
    wire m2_ack_o;
    wire m2_req_i;
    wire m2_we_i;

    // master 3 interface
    wire[`MemAddrBus] m3_addr_i;
    wire[`MemBus] m3_data_i;
    wire[`MemBus] m3_data_o;
    wire m3_ack_o;
    wire m3_req_i;
    wire m3_we_i;

    // slave 0 interface
    wire[`MemAddrBus] s0_addr_o;
    wire[`MemBus] s0_data_o;
    wire[`MemBus] s0_data_i;
    wire s0_ack_i;
    wire s0_req_o;
    wire s0_we_o;

    // slave 1 interface
    wire[`MemAddrBus] s1_addr_o;
    wire[`MemBus] s1_data_o;
    wire[`MemBus] s1_data_i;
    wire s1_ack_i;
    wire s1_req_o;
    wire s1_we_o;

    // slave 2 interface
    wire[`MemAddrBus] s2_addr_o;
    wire[`MemBus] s2_data_o;
    wire[`MemBus] s2_data_i;
    wire s2_ack_i;
    wire s2_req_o;
    wire s2_we_o;

    // slave 3 interface
    wire[`MemAddrBus] s3_addr_o;
    wire[`MemBus] s3_data_o;
    wire[`MemBus] s3_data_i;
    wire s3_ack_i;
    wire s3_req_o;
    wire s3_we_o;

    // slave 4 interface
    wire[`MemAddrBus] s4_addr_o;
    wire[`MemBus] s4_data_o;
    wire[`MemBus] s4_data_i;
    wire s4_ack_i;
    wire s4_req_o;
    wire s4_we_o;

    // slave 5 interface
    wire[`MemAddrBus] s5_addr_o;
    wire[`MemBus] s5_data_o;
    wire[`MemBus] s5_data_i;
    wire s5_ack_i;
    wire s5_req_o;
    wire s5_we_o;

    // rib
    wire rib_hold_flag_o;


    // tinyriscv
    wire[`INT_BUS] int_flag;

    // timer0
    wire timer0_int;

    // gpio
    wire[7:0] io_in;
    wire[31:0] gpio_ctrl;
    wire[31:0] gpio_data;

    assign int_flag = {7'h0, timer0_int};

    // 低电平点亮LED
    // 低电平表示已经halt住CPU
    assign halted_ind = ~spi_clk;
    wire succ;
    wire over;

    always @ (posedge clk) begin
        if (rst == `RstEnable) begin
            over_r <= 1'b1;
            succ_r <= 1'b1;
        end else begin
            //over <= ~u_tinyriscv.u_regs.regs[26];  // when = 1, run over
            //succ <= ~u_tinyriscv.u_regs.regs[27];  // when = 1, run succ, otherwise fail
            over_r <= succ;
            succ_r <= over;
        end
    end

    // pll
    pll u_pll(
        .clkin1(sys_clk),
        .clkout0(clk)
    );

    // tinyriscv处理器核模块例化
    tinyriscv u_tinyriscv(
        .clk(clk),
        .rst(rst),
        .rib_ex_addr_o(m0_addr_i),
        .rib_ex_data_i(m0_data_o),
        .rib_ex_data_o(m0_data_i),
        .rib_ex_req_o(m0_req_i),
        .rib_ex_we_o(m0_we_i),

        .rib_pc_addr_o(m1_addr_i),
        .rib_pc_data_i(m1_data_o),

        .rib_hold_flag_i(rib_hold_flag_o),


        .int_i(int_flag),
        .succ(succ),
        .over(over)
    );

    // rom模块例化
    rom u_rom(
        .clk(clk),
        .rst(rst),
        .we_i(s0_we_o),
        .addr_i(s0_addr_o),
        .data_i(s0_data_o),
        .req_i(s0_req_o),
        .data_o(s0_data_i),
        .ack_o(s0_ack_i)
    );

    // ram模块例化
    ram u_ram(
        .clk(clk),
        .rst(rst),
        .we_i(s1_we_o),
        .addr_i(s1_addr_o),
        .data_i(s1_data_o),
        .req_i(s1_req_o),
        .data_o(s1_data_i),
        .ack_o(s1_ack_i)
    );

    // timer模块例化
    timer timer_0(
        .clk(clk),
        .rst(rst),
        .data_i(s2_data_o),
        .addr_i(s2_addr_o),
        .we_i(s2_we_o),
        .data_o(s2_data_i),
        .int_sig_o(timer0_int),
        .req_i(s2_req_o),
        .ack_o(s2_ack_i)
    );

    // uart模块例化
    uart uart_0(
        .clk(clk),
        .rst(rst),
        .we_i(s3_we_o),
        .req_i(s3_req_o),
        .addr_i(s3_addr_o),
        .data_i(s3_data_o),
        .data_o(s3_data_i),
        .ack_o(s3_ack_i),
        .tx_pin(uart_tx_pin),
        .rx_pin(uart_rx_pin)
    );

    // io0
    assign gpio[0] = (gpio_ctrl[1:0] == 2'b01)? gpio_data[0]: 1'bz;
    assign io_in[0] = gpio[0];
    // io1
    assign gpio[1] = (gpio_ctrl[3:2] == 2'b01)? gpio_data[1]: 1'bz;
    assign io_in[1] = gpio[1];
    // io2
    assign gpio[2] = (gpio_ctrl[5:4] == 2'b01)? gpio_data[2]: 1'bz;
    assign io_in[2] = gpio[2];
    // io3
    assign gpio[3] = (gpio_ctrl[7:6] == 2'b01)? gpio_data[3]: 1'bz;
    assign io_in[3] = gpio[3];
    // io4
    assign gpio[4] = (gpio_ctrl[9:8] == 2'b01)? gpio_data[4]: 1'bz;
    assign io_in[4] = gpio[4];
    // io5
    assign gpio[5] = (gpio_ctrl[11:10] == 2'b01)? gpio_data[5]: 1'bz;
    assign io_in[5] = gpio[5];
    // io6
    assign gpio[6] = (gpio_ctrl[13:12] == 2'b01)? gpio_data[6]: 1'bz;
    assign io_in[6] = gpio[6];
    // io7
    assign gpio[7] = (gpio_ctrl[15:14] == 2'b01)? gpio_data[7]: 1'bz;
    assign io_in[7] = gpio[7];
    // io8
    assign gpio[8] = (gpio_ctrl[17:16] == 2'b01)? gpio_data[8]: 1'bz;
    assign io_in[8] = gpio[8];
    // io9
    assign gpio[9] = (gpio_ctrl[19:18] == 2'b01)? gpio_data[9]: 1'bz;
    assign io_in[9] = gpio[9];
    // io10
    assign gpio[10] = (gpio_ctrl[21:20] == 2'b01)? gpio_data[10]: 1'bz;
    assign io_in[10] = gpio[10];
    // io11
    assign gpio[11] = (gpio_ctrl[23:22] == 2'b01)? gpio_data[11]: 1'bz;
    assign io_in[11] = gpio[11];
    // io12
    assign gpio[12] = (gpio_ctrl[25:24] == 2'b01)? gpio_data[12]: 1'bz;
    assign io_in[12] = gpio[12];
    // io13
    assign gpio[13] = (gpio_ctrl[27:26] == 2'b01)? gpio_data[13]: 1'bz;
    assign io_in[13] = gpio[13];
    // io14
    assign gpio[14] = (gpio_ctrl[29:28] == 2'b01)? gpio_data[14]: 1'bz;
    assign io_in[14] = gpio[14];
    // io15
    assign gpio[15] = (gpio_ctrl[31:30] == 2'b01)? gpio_data[15]: 1'bz;
    assign io_in[15] = gpio[15];

    // gpio模块例化
    gpio gpio_0(
        .clk(clk),
        .rst(rst),
        .we_i(s4_we_o),
        .req_i(s4_req_o),
        .addr_i(s4_addr_o),
        .data_i(s4_data_o),
        .data_o(s4_data_i),
        .ack_o(s4_ack_i),
        .io_pin_i(io_in),
        .reg_ctrl(gpio_ctrl),
        .reg_data(gpio_data)
    );

    // spi模块例化
    spi spi_0(
        .clk(clk),
        .rst(rst),
        .data_i(s5_data_o),
        .addr_i(s5_addr_o),
        .we_i(s5_we_o),
        .req_i(s5_req_o),
        .data_o(s5_data_i),
        .ack_o(s5_ack_i),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_ss(spi_ss),
        .spi_clk(spi_clk)
    );

    // rib模块例化
    rib u_rib(
        .clk(clk),
        .rst(rst),

        // master 0 interface
        .m0_addr_i(m0_addr_i),
        .m0_data_i(m0_data_i),
        .m0_data_o(m0_data_o),
        .m0_ack_o(m0_ack_o),
        .m0_req_i(m0_req_i),
        .m0_we_i(m0_we_i),

        // master 1 interface
        .m1_addr_i(m1_addr_i),
        .m1_data_i(`ZeroWord),
        .m1_data_o(m1_data_o),
        .m1_ack_o(m1_ack_o),
        .m1_req_i(`RIB_REQ),
        .m1_we_i(`WriteDisable),

        // master 2 interface
        .m2_addr_i(m2_addr_i),
        .m2_data_i(m2_data_i),
        .m2_data_o(m2_data_o),
        .m2_ack_o(m2_ack_o),
        .m2_req_i(m2_req_i),
        .m2_we_i(m2_we_i),

        // master 3 interface
        .m3_addr_i(m3_addr_i),
        .m3_data_i(m3_data_i),
        .m3_data_o(m3_data_o),
        .m3_ack_o(m3_ack_o),
        .m3_req_i(m3_req_i),
        .m3_we_i(m3_we_i),

        // slave 0 interface
        .s0_addr_o(s0_addr_o),
        .s0_data_o(s0_data_o),
        .s0_data_i(s0_data_i),
        .s0_ack_i(s0_ack_i),
        .s0_req_o(s0_req_o),
        .s0_we_o(s0_we_o),

        // slave 1 interface
        .s1_addr_o(s1_addr_o),
        .s1_data_o(s1_data_o),
        .s1_data_i(s1_data_i),
        .s1_ack_i(s1_ack_i),
        .s1_req_o(s1_req_o),
        .s1_we_o(s1_we_o),

        // slave 2 interface
        .s2_addr_o(s2_addr_o),
        .s2_data_o(s2_data_o),
        .s2_data_i(s2_data_i),
        .s2_ack_i(s2_ack_i),
        .s2_req_o(s2_req_o),
        .s2_we_o(s2_we_o),

        // slave 3 interface
        .s3_addr_o(s3_addr_o),
        .s3_data_o(s3_data_o),
        .s3_data_i(s3_data_i),
        .s3_ack_i(s3_ack_i),
        .s3_req_o(s3_req_o),
        .s3_we_o(s3_we_o),

        // slave 4 interface
        .s4_addr_o(s4_addr_o),
        .s4_data_o(s4_data_o),
        .s4_data_i(s4_data_i),
        .s4_ack_i(s4_ack_i),
        .s4_req_o(s4_req_o),
        .s4_we_o(s4_we_o),

        // slave 5 interface
        .s5_addr_o(s5_addr_o),
        .s5_data_o(s5_data_o),
        .s5_data_i(s5_data_i),
        .s5_ack_i(s5_ack_i),
        .s5_req_o(s5_req_o),
        .s5_we_o(s5_we_o),

        .hold_flag_o(rib_hold_flag_o)
    );

    // 串口下载模块例化
    uart_debug u_uart_debug(
        .clk(clk),
        .rst(rst),
        .debug_en_i(uart_debug_pin),
        .req_o(m3_req_i),
        .mem_we_o(m3_we_i),
        .mem_addr_o(m3_addr_i),
        .mem_wdata_o(m3_data_i),
        .mem_rdata_i(m3_data_o)
    );


endmodule
