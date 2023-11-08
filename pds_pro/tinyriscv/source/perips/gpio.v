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

// Modified by Darkbe on 2023-11-03: add gpio to 16;
// GPIO模块
module gpio(
    
    input wire clk,
	input wire rst,

    input wire we_i,
    input wire req_i,
    input wire[31:0] addr_i,
    input wire[31:0] data_i,

    output reg[31:0] data_o,
    output reg ack_o,

    input wire[15:0] io_pin_i,
    output wire[31:0] reg_ctrl,
    output wire[31:0] reg_data

    );


    // GPIO控制寄存器
    localparam GPIO_CTRL = 4'h0;
    // GPIO数据寄存器
    localparam GPIO_DATA = 4'h4;

    // 每2位控制1个IO的模式，最多支持16个IO
    // 0: 高阻，1：输出，2：输入
    reg[31:0] gpio_ctrl;
    // 输入输出数据
    reg[31:0] gpio_data;


    assign reg_ctrl = gpio_ctrl;
    assign reg_data = gpio_data;


    // 写寄存器
    always @ (posedge clk) begin
        if (rst == 1'b0) begin
            gpio_data <= 32'h0;
            gpio_ctrl <= 32'h0;
        end else begin
            if (we_i == 1'b1) begin
                case (addr_i[3:0])
                    GPIO_CTRL: begin
                        gpio_ctrl <= data_i;
                    end
                    GPIO_DATA: begin
                        gpio_data <= data_i;
                    end
                endcase
            end else begin
                if (gpio_ctrl[1:0] == 2'b10) begin
                    gpio_data[0] <= io_pin_i[0];
                end
                if (gpio_ctrl[3:2] == 2'b10) begin
                    gpio_data[1] <= io_pin_i[1];
                end
                if (gpio_ctrl[5:4] == 2'b10) begin
                    gpio_data[2] <= io_pin_i[2];
                end
                if (gpio_ctrl[7:6] == 2'b10) begin
                    gpio_data[3] <= io_pin_i[3];
                end
                if (gpio_ctrl[9:8] == 2'b10) begin
                    gpio_data[4] <= io_pin_i[4];
                end
                if (gpio_ctrl[11:10] == 2'b10) begin
                    gpio_data[5] <= io_pin_i[5];
                end
                if (gpio_ctrl[13:12] == 2'b10) begin
                    gpio_data[6] <= io_pin_i[6];
                end
                if (gpio_ctrl[15:14] == 2'b10) begin
                    gpio_data[7] <= io_pin_i[7];
                end
                if (gpio_ctrl[17:16] == 2'b10) begin
                    gpio_data[8] <= io_pin_i[8];
                end
                if (gpio_ctrl[19:18] == 2'b10) begin
                    gpio_data[9] <= io_pin_i[9];
                end
                if (gpio_ctrl[21:20] == 2'b10) begin
                    gpio_data[10] <= io_pin_i[10];
                end
                if (gpio_ctrl[23:22] == 2'b10) begin
                    gpio_data[11] <= io_pin_i[11];
                end
                if (gpio_ctrl[25:24] == 2'b10) begin
                    gpio_data[12] <= io_pin_i[12];
                end
                if (gpio_ctrl[27:26] == 2'b10) begin
                    gpio_data[13] <= io_pin_i[13];
                end
                if (gpio_ctrl[29:28] == 2'b10) begin
                    gpio_data[14] <= io_pin_i[14];
                end
                if (gpio_ctrl[31:30] == 2'b10) begin
                    gpio_data[15] <= io_pin_i[15];
                end
            end
        end
    end

    // 读寄存器
    always @ (*) begin
        if (rst == 1'b0) begin
            data_o = 32'h0;
        end else begin
            case (addr_i[3:0])
                GPIO_CTRL: begin
                    data_o = gpio_ctrl;
                end
                GPIO_DATA: begin
                    data_o = gpio_data;
                end
                default: begin
                    data_o = 32'h0;
                end
            endcase
        end
    end

endmodule
