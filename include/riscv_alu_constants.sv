`ifndef RISCV_ALU_CONSTANTS
`define RISCV_ALU_CONSTANTS

localparam[3:0] ALUOP_AND = 4'b0000;
localparam[3:0] ALUOP_OR = 4'b0001;
localparam[3:0] ALUOP_ADD = 4'b0010;
localparam[3:0] ALUOP_SUB = 4'b0110;
localparam[3:0] ALUOP_LESS = 4'b0111;
localparam[3:0] ALUOP_SRL = 4'b1000;
localparam[3:0] ALUOP_SLL = 4'b1001;
localparam[3:0] ALUOP_SRA = 4'b1010;
localparam[3:0] ALUOP_XOR = 4'b1101;


`endif // RISCV_ALU_CONSTANTS
