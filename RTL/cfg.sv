// cfg.sv — runtime configuration register bank

module cfg(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        busy,
    input  logic [15:0] cfg_img_width,
    input  logic [15:0] cfg_img_height,
    input  logic        cfg_relu_en,
    input  logic [4:0]  cfg_shift_amt,
    input  logic        cfg_round_en,

    output logic [15:0] img_width, 
    output logic [15:0] img_height,
    output logic        relu_en,   
    output logic [4:0]  shift_amt, 
    output logic        round_en   
);


    always_ff @(posedge clk) begin
        if (!rst_n) begin
            img_width  <= 16'd32;  
            img_height <= 16'd32;
            relu_en    <= 1'b0;    
            shift_amt  <= 5'd0;    
            round_en   <= 1'b0;    
        end
        else if(!busy) begin
            img_width  <= cfg_img_width;
            img_height <= cfg_img_height;
            relu_en    <= cfg_relu_en;
            shift_amt  <= cfg_shift_amt;
            round_en   <= cfg_round_en;
        end
        // else (busy = 1, PROCESSING) → no branch taken → registers HOLD = read-only
    end

endmodule