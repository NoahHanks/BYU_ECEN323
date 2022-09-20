`ifndef RISCV_BASIC_PIPELINE_CONSTANTS
`define RISCV_BASIC_PIPELINE_CONSTANTS

 localparam[6:0] R_TYPE_OP_CODE = 7'b0110011; 
 localparam[6:0] I_TYPE_OP_CODE = 7'b0010011;
 localparam[6:0] ILOAD_TYPE_OP_CODE = 7'b0000011;  
 localparam[6:0] S_TYPE_OP_CODE = 7'b0100011; 
 localparam[6:0] SB_TYPE_OP_CODE = 7'b1100011; 

 localparam[2:0] FUNCT3_OR = 3'b110;
 localparam[2:0] FUNCT3_SLT = 3'b010;
 localparam[2:0] FUNCT3_XOR = 3'b100;
 localparam[2:0] FUNCT3_AND = 3'b111;
 localparam[2:0] FUNCT3_SUB = 3'b000;
 localparam[2:0] FUNCT3_ADD = 3'b000;
 localparam[2:0] FUNCT3_SLL = 3'b001;
 localparam[2:0] FUNCT3_SRL = 3'b101;
 localparam[2:0] FUNCT3_SRA = 3'b101;
 localparam[6:0] FUNCT7_SUB = 7'b0100000;
 localparam[6:0] FUNCT7_ADD = 7'b0000000;
 localparam[6:0] FUNCT7_SRL = 7'b0100000;
 localparam[6:0] FUNCT7_SRA = 7'b0000000;
 
 localparam[31:0] NOP_CONST = 32'h00000013;
 localparam[31:0] EMPTY_32 = 32'h00000000;
 
 localparam[1:0] FORWARD_0 = 2'b00;
 localparam[1:0] FORWARD_1 = 2'b01;
 localparam[1:0] FORWARD_2 = 2'b10;

localparam[31:0] SINGLE_INSTRUCTION_OFFSET = 32'h00000004;
localparam[6:0] I_TYPE_OP = 7'b0010011;
localparam[6:0] LOAD_OP = 7'b0000011;
localparam[6:0] R_TYPE_OP = 7'b0110011;
localparam[6:0] U_TYPE_OP = 7'b0110111;
localparam[6:0] S_TYPE_OP = 7'b0100011;
localparam[6:0] SB_TYPE_OP = 7'b1100011;
localparam[6:0] JAL_OP = 7'b1101111;
localparam[6:0] JALR_OP = 7'b1100111;

localparam[2:0] ADD_SUB_F3 = 3'b000;

localparam[2:0] SL_F3 = 3'b001;
localparam[6:0] LOGICAL_F7 = 7'b0000000;

localparam[2:0] SLT_F3 = 3'b010;
localparam[2:0] SLTU_F3 = 3'b011;
localparam[2:0] XOR_F3 = 3'b100;
localparam[2:0] SR_F3 = 3'b101;
localparam[2:0] OR_F3 = 3'b110;
localparam[2:0] AND_F3 = 3'b111;
localparam[6:0] SUB_F7 = 7'b0100000;

localparam[2:0] BEQ = 3'b000;
localparam[2:0] BNE = 3'b001;
localparam[2:0] BLT = 3'b100;
localparam[2:0] BGE = 3'b101;

`endif // RISCV_BASIC_PIPELINE_CONSTANTS
