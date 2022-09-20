`timescale 1 ns / 1 ps 
 `include "riscv_datapath_constants.sv"
 `include "riscv_alu_constants.sv"
/***************************************************************************
* 
* File: riscv_multicycle.sv
*
* Author: Noah Hanks
* Class: ECEN 323, Winter Semester 2022
* Date: 2/8/2022
*
* Module: riscv_multicycle
*
* Description: This module takes all the different inputs and outputs and puts them through a state machine.
    The state machine has 5 states and it clocks bath and forth and determines the values of the muxes, outputs
    and so forth.
*    
*	
****************************************************************************/

module riscv_multicycle #(parameter INITIAL_PC = 32'h00400000)(clk, rst, instruction, dReadData, PC, 
dAddress, dWriteData, MemRead, MemWrite, WriteBackData);

    input wire logic clk, rst;
    input wire logic [31:0] instruction, dReadData;
    output logic MemRead, MemWrite;
    output logic [31:0] PC, dAddress, dWriteData, WriteBackData;
    
    logic ALUSrc, MemtoReg, RegWrite, loadPC, PCSrc, Zero;
    logic [3:0] ALUCtrl;
    logic [6:0] opcode, funct7;
    logic [2:0] funct3;
    
    //implments the simple datapath module.
     riscv_simple_datapath #INITIAL_PC datapath(.clk(clk), .rst(rst), .PCSrc(PCSrc), .ALUSrc(ALUSrc), 
     .RegWrite(RegWrite), .MemtoReg(MemtoReg), .loadPC(loadPC), .instruction(instruction), .dReadData(dReadData), 
     .ALUCtrl(ALUCtrl), .Zero(Zero), .PC(PC), .dAddress(dAddress), .dWriteData(dWriteData), .WriteBackData(WriteBackData));
    
    typedef enum logic[2:0] {IF, ID, EX, MEM, WB, ERR='X} StateType;
    StateType cs, ns;
    
    assign opcode = {instruction[6:0]};
    assign funct3 = {instruction[14:12]};
    assign funct7 = {instruction[31:25]};
    
    assign ALUSrc = ((opcode != R_TYPE_OP_CODE) && (opcode != SB_TYPE_OP_CODE)) ? 1 : 0;
    assign MemRead = ((opcode == ILOAD_TYPE_OP_CODE) && (cs == MEM)) ? 1 : 0;
    assign MemWrite = ((opcode == S_TYPE_OP_CODE) && (cs == MEM)) ? 1 : 0;
    assign MemtoReg = ((opcode == ILOAD_TYPE_OP_CODE) && (cs == WB)) ? 1 : 0;
    assign RegWrite = ((opcode != S_TYPE_OP_CODE) && (opcode != SB_TYPE_OP_CODE) && (cs == WB)) ? 1 : 0;
    assign loadPC = (cs == WB) ? 1 : 0;
    assign PCSrc = (Zero && (opcode == SB_TYPE_OP_CODE) && (cs == WB)) ? 1 : 0;
    
    //This block decides the ALUCtrl based on the opcode, funct3, and funct7
    always_comb begin
        if((opcode == ILOAD_TYPE_OP_CODE) || (opcode == S_TYPE_OP_CODE)) // for load/store, add
            ALUCtrl = ALUOP_ADD;
        else if(opcode == SB_TYPE_OP_CODE)    //for branch, subtract
            ALUCtrl = ALUOP_SUB;
        else if((opcode == R_TYPE_OP_CODE) || (opcode == I_TYPE_OP_CODE)) begin
            if(funct3 == FUNCT3_OR)        // for or/ori
                ALUCtrl = ALUOP_OR;
            else if(funct3 == FUNCT3_SLT) // for slt/slti
                ALUCtrl = ALUOP_LES;
            else if(funct3 == FUNCT3_XOR) // for xor/xori
                ALUCtrl = ALUOP_XOR;
            else if(funct3 == FUNCT3_AND) // for and/andi
                ALUCtrl = ALUOP_AND;                
            else if((funct3 == FUNCT3_SUB) && (funct7 == FUNCT7_SUB) && (opcode == R_TYPE_OP_CODE)) // for sub
                ALUCtrl = ALUOP_SUB;
            else if((funct3 == FUNCT3_ADD) && (opcode == I_TYPE_OP_CODE)) // for sub
                ALUCtrl = ALUOP_ADD;
            else if((funct3 == FUNCT3_ADD) && (funct7 == FUNCT7_ADD)) // for add/addi
                ALUCtrl = ALUOP_ADD;
            else if(funct3 == FUNCT3_SLL) // for sll/slli
                ALUCtrl = ALUOP_SLL;
            else if((funct3 == FUNCT3_SRL) && (funct7 == FUNCT7_SRL)) // for srl/srli
                ALUCtrl = ALUOP_SRL;
            else if((funct3 == FUNCT3_SRA) && (funct7 == FUNCT7_SRA)) // for sra/srai
                ALUCtrl = ALUOP_SRA;
            else
                ALUCtrl = ALUOP_ADD;
            end
        else
            ALUCtrl = ALUOP_ADD;
    end
    
    // block to change the next state to the current
    always_ff@(posedge clk)
        cs <= ns;

    // big state machine 
    always_comb begin
           ns = ERR;
           if(rst)
                ns = IF;
           else
           case(cs)
                IF: 
                    ns = ID;
                ID:
                    ns = EX;   
                EX: 
                    ns = MEM;   
                MEM:              
                    ns = WB;   
                WB: 
                    ns = IF; 
                default:
                    ns = IF; 
           endcase
    end
         
         
    endmodule