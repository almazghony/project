module top#(
    int unsigned N = 5,
    int unsigned KERNEL_ADDR_WIDTH = $clog2(N*N)
)(
    input   logic                           clk,
    input   logic                           rst_n,
    input   logic                           start,
    input   logic                           kernel_we,
    input   logic [KERNEL_ADDR_WIDTH:0]     kernel_addr,
    input   logic                           kernel_data,
    input   logic [7:0]                     pixel_in,
    input   logic                           pixel_in_valid,
    output  logic [15:0]                    pixel_out,
    output  logic                           pixel_out_valid,
    output  logic                           done,
    output  logic                           busy,
    output  logic [15:0]                    cfg_img_width
    output  logic [15:0]                    cfg_img_heigh
    output  logic                           cfg_relu_en,
    output  logic [4:0]                     cfg_shift_amt
    output  logic                           cfg_round_en,

);  


endmodule