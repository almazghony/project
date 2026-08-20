module kernel_mem#(
    int N           = 5,
    int KERNEL_ADDR_WIDTH = $clog2(N*N)
)(
    input   logic                         clk,
    input   logic                         rst_n,
    input   logic                         kernel_we,
    input   logic [KERNEL_ADDR_WIDTH-1:0] kernel_addr,
    input   logic [7:0]                   kernel_data,
    input   logic                         processing_en,
    output  logic [7:0]                   kernel_coeff[N][N]
);

    logic signed [7:0] mem [0:N*N-1];


    always_ff @(posedge clk) begin
        //Kernel contents are undefined after reset.
        //The kernel must always be programmed by software/testbench before START
        //the accelerator never assumes a default kernel.
            if(!processing_en && kernel_we)
                    mem[kernel_addr] <= kernel_data;
    end

    
    generate;
        for(genvar r = 0; r < N, r++) begin
            for(genvar c = 0; c < N; c++) begin
                kernel_coeff[r][c] = mem[r*N + c];
            end
        end
    endgenerate
endmodule