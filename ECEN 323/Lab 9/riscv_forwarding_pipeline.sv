`timescale 1 ns / 1 ps 
 `include "riscv_alu_constants.sv"
 `include "riscv_basic_pipeline_constants.sv"
/***************************************************************************
* 
* File: riscv_forwarding_pipeline.sv
*
* Author: Noah Hanks
* Class: ECEN 323, Winter Semester 2022
* Date: 2/22/2022
*
* Module: riscv_forwarding_pipeline
*
* Description: This module pipelines 5 different instructions at once through the 
*    datapath and uses five seperate stages to decide what to do with each instruction and when.
*	
****************************************************************************/

module riscv_forwarding_pipeline #(parameter INITIAL_PC = 32'h00400000)(clk, rst, PC, iMemRead, instruction, ALUResult,
dAddress, dWriteData, dReadData,  MemRead, MemWrite, WriteBackData);

    input wire logic clk, rst;
    input wire logic [31:0] instruction, dReadData;
    output logic MemRead, MemWrite, iMemRead;
    output logic [31:0] PC, ALUResult, dAddress, dWriteData, WriteBackData;

    //HAZARD DETECTION LOGIC
    logic load_use_hazard, branch_mem_taken, branch_wb_taken, PCWrite;

    //Forwarding Signals
    logic [1:0] ForwardA, ForwardB;
    logic [31:0] op1_input, op2_input;
    
    // IF signals
    logic [4:0] if_rs1, if_rs2, if_rd;
    logic [31:0] if_PC, if_instruction;
    
    // ID signals
    logic id_ALUSrc, id_MemWrite, id_MemRead, id_Branch, id_MemtoReg, id_RegWrite;
    logic [3:0] id_ALUCtrl;
    logic [2:0] id_funct3;
    logic [4:0] id_writereg;
    logic [4:0] id_rs1, id_rs2, id_rd;
    logic [6:0] id_opcode, id_funct7;
    logic [31:0] id_PC, id_instruction, id_branch_offset, id_ReadData1, id_ReadData2, id_s_type_immediate, id_instruction_ext;
    
    // EX signals
    logic ex_ALUSrc, ex_MemWrite, ex_MemRead, ex_Branch, ex_Zero, ex_MemtoReg, ex_RegWrite;
    logic [3:0] ex_ALUCtrl;
    logic [4:0] ex_rs1, ex_rs2, ex_rd;
    logic [6:0] ex_opcode;
    logic [4:0] ex_writereg;
    logic [31:0] ex_PC, ex_branch_offset, ex_ALUresult, ex_ReadData1, ex_ReadData2, ex_op2, ex_instruction_ext, ex_s_type_immediate;
    
    // MEM signals
    logic mem_PCSrc, mem_MemtoReg, mem_RegWrite, mem_MemWrite, mem_MemRead, mem_Zero, mem_Branch;
    logic [4:0] mem_writereg;
    logic [4:0] mem_rs1, mem_rs2, mem_rd;
    logic [31:0] mem_ALUresult, mem_branch_offset, mem_ReadData2, mem_PC;
    
    // WB signals
    logic wb_PCSrc, wb_MemtoReg, wb_RegWrite;
    logic [4:0] wb_writereg;
    logic [4:0] wb_rs1, wb_rs2, wb_rd;
    logic [31:0] wb_ALUresult, wb_WriteBackData;

    assign load_use_hazard = (ex_MemRead == 1 && ((id_rd == if_rs1 && if_rs1 != 0) || (id_rd == if_rs2 && if_rs2 != 0)) &&  !(mem_Branch && mem_Zero) && ex_opcode == ILOAD_TYPE_OP_CODE) ? 1 : 0;
    assign PCWrite = load_use_hazard ? 0 : 1;
//    assign instruction = load_use_hazard ? NOP_CONST : instruction;

    always_comb begin
        if((id_RegWrite || ex_RegWrite) && ex_rd != 0 && ex_rd == id_rs1)
            ForwardA = 2'b10;
        else if(mem_RegWrite && mem_rd != 0 && mem_rd == id_rs1 && !(ex_RegWrite && ex_rd != 0 && ex_rd == id_rs1))
            ForwardA = 2'b01;
        else
            ForwardA = 2'b00;
        if(id_RegWrite && ex_rd != 0 && ex_rd == id_rs2)
            ForwardB = 2'b10;
        else if(mem_RegWrite && mem_rd != 0 && mem_rd == id_rs2 && !(ex_RegWrite && ex_rd != 0 && ex_rd == id_rs2))
            ForwardB = 2'b01;
        else
            ForwardB = 2'b00;
    end

//////////////////////////////////////////////////////////////////////
// IF: Instruction Fetch
//////////////////////////////////////////////////////////////////////	

// Always ff block for the IF pipelined signals
always_ff@(posedge clk) begin
        if(rst) begin
            PC <= INITIAL_PC;
            if_PC <= INITIAL_PC;
            id_instruction <= NOP_CONST;
        end
        else begin
            if_PC <= PC;
            id_instruction <= instruction; 
            if(PCWrite)
                if(!mem_PCSrc)
                    PC <= PC + 4;
                else
                    PC <= mem_PC + ex_branch_offset;
            else
                PC <= PC;
        end
    end
    
assign if_rd = instruction[11:7];
assign if_rs1 = instruction[19:15];
assign if_rs2 = instruction[24:20];

//////////////////////////////////////////////////////////////////////
// ID: Instruction Decode
//////////////////////////////////////////////////////////////////////	

                    
// Instance the regfile module
    regfile regfile0 (.clk(clk), .readReg1(instruction[19:15]), 
        .readReg2(instruction[24:20]), .writeReg(wb_writereg), .writeData(WriteBackData), 
        .write(wb_RegWrite), .readData1(id_ReadData1), .readData2(id_ReadData2));

// Always ff block for the ID pipelined signals
always_ff@(posedge clk) begin
    
        if(rst || instruction == NOP_CONST || load_use_hazard) begin
            id_PC <= INITIAL_PC;
            id_branch_offset <= EMPTY_32;
            id_writereg <= 5'b00000;
            id_rs1 <= 5'b00000;
            id_rs2 <= 5'b00000;
            id_rd <= 5'b00000;
            end
        else begin
            if(PCWrite)
                id_PC <= PC;
            else
                id_PC <= id_PC;
            id_rs1 <= instruction[19:15];
            if(id_opcode != I_TYPE_OP_CODE)
                id_rs2 <= instruction[24:20];
            else
                id_rs2 <= 5'b00000;
            id_rd <= instruction[11:7];
            id_branch_offset <= {{19 {instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            end
     
    end
    
    
    assign id_opcode = {instruction[6:0]};
    assign id_funct3 = {instruction[14:12]};
    assign id_funct7 = {instruction[31:25]};

    assign id_ALUSrc = ((id_opcode != R_TYPE_OP_CODE) && (id_opcode != SB_TYPE_OP_CODE)) ? 1 : 0;
    assign id_MemWrite = (id_opcode == S_TYPE_OP_CODE) ? 1 : 0;
    assign id_MemRead = (id_opcode == ILOAD_TYPE_OP_CODE) ? 1 : 0;
    assign id_Branch = (id_opcode == SB_TYPE_OP_CODE) ? 1 : 0; 
    assign id_MemtoReg = (id_opcode == ILOAD_TYPE_OP_CODE) ? 1 : 0;
    assign id_RegWrite = ((id_opcode != S_TYPE_OP_CODE) && (id_opcode != SB_TYPE_OP_CODE)) ? 1 : 0;
    
    assign id_s_type_immediate = {{20 {instruction[31]}}, instruction[31:25], instruction[11:7]};
    assign id_instruction_ext = {{20 {instruction[31]}}, instruction[31:20]};
    
//This block decides the ALUCtrl based on the opcode, funct3, and funct7
    always_comb begin
        if((id_opcode == ILOAD_TYPE_OP_CODE) || (id_opcode == S_TYPE_OP_CODE)) // for load/store, add
            id_ALUCtrl = ALUOP_ADD;
        else if(id_opcode == SB_TYPE_OP_CODE)    //for branch, subtract
            id_ALUCtrl = ALUOP_SUB;
        else if((id_opcode == R_TYPE_OP_CODE) || (id_opcode == I_TYPE_OP_CODE)) begin
            if(id_funct3 == FUNCT3_OR)        // for or/ori
                id_ALUCtrl = ALUOP_OR;
            else if(id_funct3 == FUNCT3_SLT) // for slt/slti
                id_ALUCtrl = ALUOP_LES;
            else if(id_funct3 == FUNCT3_XOR) // for xor/xori
                id_ALUCtrl = ALUOP_XOR;
            else if(id_funct3 == FUNCT3_AND) // for and/andi
                id_ALUCtrl = ALUOP_AND;                
            else if((id_funct3 == FUNCT3_SUB) && (id_funct7 == FUNCT7_SUB) && (id_opcode == R_TYPE_OP_CODE)) // for sub
                id_ALUCtrl = ALUOP_SUB;
            else if((id_funct3 == FUNCT3_ADD) && (id_opcode == I_TYPE_OP_CODE)) // for sub
                id_ALUCtrl = ALUOP_ADD;
            else if((id_funct3 == FUNCT3_ADD) && (id_funct7 == FUNCT7_ADD)) // for add/addi
                id_ALUCtrl = ALUOP_ADD;
            else if(id_funct3 == FUNCT3_SLL) // for sll/slli
                id_ALUCtrl = ALUOP_SLL;
            else if((id_funct3 == FUNCT3_SRL) && (id_funct7 == FUNCT7_SRL)) // for srl/srli
                id_ALUCtrl = ALUOP_SRL;
            else if((id_funct3 == FUNCT3_SRA) && (id_funct7 == FUNCT7_SRA)) // for sra/srai
                id_ALUCtrl = ALUOP_SRA;
            else
                id_ALUCtrl = ALUOP_ADD;
            end
        else
            id_ALUCtrl = ALUOP_ADD;
    end

//////////////////////////////////////////////////////////////////////
// EX: Execute
//////////////////////////////////////////////////////////////////////	

always_comb begin
    case(ForwardA)
        2'b00:
            op1_input = id_ReadData1;
        2'b10:
            op1_input = mem_ALUresult;
        2'b01:
            op1_input = WriteBackData;
    endcase
    case(ForwardB)
        2'b00:
            op2_input = ex_op2;
        2'b10:
            op2_input = mem_ALUresult;
        2'b01:
            op2_input = WriteBackData;
    endcase
end

// Instance the ALU module  
    alu alu0 (.op1(op1_input), .op2(op2_input), .alu_op(ex_ALUCtrl), .zero(ex_Zero), .result(ex_ALUresult));
    
    assign ALUResult = ex_ALUresult;

// Always ff block for the EX pipelined signals
always_ff@(posedge clk) begin

        if(rst || branch_mem_taken || branch_wb_taken) begin
            ex_PC <= ex_PC;
//            ex_ALUresult <= EMPTY_32;
            ex_rs1 <= 5'b00000;
            ex_rs2 <= 5'b00000;
            ex_rd <= 5'b00000;
            ex_writereg <= 5'b00000;
            ex_MemWrite <= 0;
            ex_MemRead <= 0;
            ex_MemtoReg <= 0;
            ex_RegWrite <= 1;
            ex_Branch <= 0;
            ex_ALUSrc <= 0;
            ex_ALUCtrl <= ALUOP_ADD;
            ex_branch_offset <= EMPTY_32;
            ex_ReadData1 <= EMPTY_32;
            ex_ReadData2 <= EMPTY_32;
            ex_opcode <= 7'b0000000;
            ex_instruction_ext <= EMPTY_32;
            ex_s_type_immediate <= EMPTY_32;
            end
            
        else begin
            ex_rs1 <= id_rs1;
            ex_rs2 <= id_rs2;
            ex_rd <= id_rd;
            ex_PC <= id_PC;
            ex_writereg <= instruction[11:7];
            ex_MemWrite <= id_MemWrite;
//            if(id_rd == 5'b00000)
//                ex_MemRead <= 0;
//            else
                ex_MemRead <= id_MemRead;
            
            
            ex_MemtoReg <= id_MemtoReg;
            ex_Branch <= id_Branch;
            ex_ALUSrc <= id_ALUSrc;
            ex_ALUCtrl <= id_ALUCtrl;
            ex_branch_offset <= id_branch_offset;
            ex_ReadData1 <= id_ReadData1;
            ex_ReadData2 <= id_ReadData2;
            ex_opcode <= id_opcode;
            ex_instruction_ext <= id_instruction_ext;
            ex_s_type_immediate <= id_s_type_immediate;
            ex_RegWrite <= id_RegWrite;
            
            end
            
    
    
    
    end
    
// Checks if the instruction type is S-type and sets ReadData2 to the sign extended immediate if so.    
    always_comb
        if(ex_ALUSrc)
            if(ex_opcode == S_TYPE_OP_CODE)
                ex_op2 = ex_s_type_immediate;    
            else
                ex_op2 = ex_instruction_ext;
        else   
            ex_op2 = id_ReadData2;

//////////////////////////////////////////////////////////////////////
// MEM: Memory
//////////////////////////////////////////////////////////////////////	

// Always ff block for the MEM pipelined signals
always_ff@(posedge clk) begin
    
    if(rst || branch_mem_taken) begin
        mem_rs1 <= 5'b00000;
        mem_rs2 <= 5'b00000;
        mem_rd <= 5'b00000;
        mem_writereg <= 5'b00000;
        mem_MemRead <= 0;
        mem_MemWrite <= 0;
        mem_RegWrite <= 1;
        mem_PCSrc <= 0;
        mem_Branch <= 0;
        mem_ReadData2 <= EMPTY_32;
        mem_Zero <= 0;
        mem_MemtoReg <= 0;
        mem_ALUresult <= EMPTY_32;
        mem_branch_offset <= EMPTY_32;
        mem_PC <= EMPTY_32;
        end
    else begin
        mem_PC <= ex_PC;
        mem_rs1 <= ex_rs1;
        mem_rs2 <= ex_rs2;
        mem_rd <= ex_rd;
        mem_writereg <= ex_writereg;
        if(if_rd == 5'b00000)
            mem_MemWrite <= 0;
        else
            mem_MemWrite <= ex_MemWrite;
        if(id_rd == 5'b00000) begin
                mem_MemRead <= 0;
//                mem_MemWrite <= 0;
//                if(id_opcode == S_TYPE_OP_CODE)
//                    mem_MemWrite <= 1;
                end
        else begin
//                ex_MemRead <= id_MemRead;
                mem_MemRead <= ex_MemRead;
//                mem_MemWrite <= ex_MemWrite;
                end
        mem_RegWrite <= ex_RegWrite;
        mem_PCSrc <= (ex_Zero && ex_Branch);
        mem_Branch <= ex_Branch;
        mem_ReadData2 <= ex_ReadData2;
        mem_Zero <= ex_Zero;
        mem_MemtoReg <= ex_MemtoReg;
        mem_ALUresult <= ex_ALUresult;
        mem_branch_offset <= ex_branch_offset;
        end
          
    end
    
    assign dWriteData = ex_ReadData2;
    assign dAddress = mem_ALUresult;
    assign MemRead = mem_MemRead;
    assign MemWrite = mem_MemWrite;    
    assign branch_mem_taken = mem_PCSrc;
    assign iMemRead = load_use_hazard ? 0 : 1;

//////////////////////////////////////////////////////////////////////
// WB: Write Back
//////////////////////////////////////////////////////////////////////

// Always ff block for the WB pipelined signals	
always_ff@(posedge clk) begin

     if(rst) begin
        wb_rs1 <= 5'b00000;
        wb_rs2 <= 5'b00000;
        wb_rd <= 5'b00000;
         wb_writereg <= 5'b00000;
         wb_WriteBackData <= EMPTY_32;
         wb_MemtoReg <= 0;
         wb_RegWrite <= 1;
         wb_PCSrc <= 0;
         end
     else begin
        wb_rs1 <= mem_rs1;
        wb_rs2 <= mem_rs2;
        wb_rd <= mem_rd;
         wb_PCSrc <= mem_PCSrc;
         wb_writereg <= mem_writereg;
         wb_MemtoReg <= mem_MemtoReg;
         wb_RegWrite <= mem_RegWrite;
         wb_ALUresult <= mem_ALUresult;
         wb_WriteBackData <= mem_ALUresult;
        
         end
     end
 
 assign WriteBackData = wb_MemtoReg ? dReadData : wb_WriteBackData;
 assign branch_wb_taken = wb_PCSrc;

endmodule






