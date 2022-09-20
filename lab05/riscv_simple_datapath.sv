`timescale 1 ns / 1 ps 
`include "riscv_datapath_constants.sv"
/***************************************************************************
* 
* File: riscv_simple_datapath.sv
*
* Author: Noah Hanks
* Class: ECEN 323, Winter Semester 2022
* Date: 2/2/2022
*
* Module: riscv_simple_datapath
*
* Description: This module takes input from the PCSrc, ALU, registers and memory to do different functions
    that are used in a risc-v processor such as adds, sotres, branches, etc.
*    
*	
****************************************************************************/

module riscv_simple_datapath #(parameter INITIAL_PC = 32'h00400000)(clk, rst, PCSrc, ALUSrc, RegWrite, MemtoReg, loadPC, 
    instruction, dReadData, ALUCtrl, Zero, PC, dAddress, dWriteData, WriteBackData);
    
//    parameter INITIAL_PC = 32'h00400000;
    
    input wire logic clk, rst, PCSrc, ALUSrc, RegWrite, MemtoReg, loadPC;
    input wire logic [31:0] instruction, dReadData;
    input wire logic [3:0] ALUCtrl;
    output logic Zero;
    output logic [31:0] PC, dAddress, dWriteData, WriteBackData;
    
    logic [31:0] ReadData1, ReadData2, instruction_ext, ALUresult, op2, s_type_immediate;
//    logic [12:0] branch_offset;
    logic [31:0] branch_offset;
    
    assign s_type_immediate = {{20 {instruction[31]}}, instruction[31:25], instruction[11:7]};
    assign instruction_ext = {{20 {instruction[31]}}, instruction[31:20]};
    assign branch_offset = {{19 {instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};

    // Checks if PCSrc is ready for a branch then adds the offset to PC is so. Also does the reset case.
    always_ff@(posedge clk) begin
        if(rst)
            PC <= INITIAL_PC;
        if(loadPC & PCSrc & Zero)
             PC <= PC + branch_offset;
        else if(loadPC)
             PC <= PC + 4;
        end

    // Instance the regfile module
    regfile regfile0 (.clk(clk), .readReg1(instruction[19:15]), 
        .readReg2(instruction[24:20]), .writeReg(instruction[11:7]), .writeData(WriteBackData), 
        .write(RegWrite), .readData1(ReadData1), .readData2(ReadData2));
        
    // Checks if the instruction type is S-type and sets ReadData2 to the sign extended immediate if so.    
    always_comb
        if(ALUSrc)
            if(instruction[6:0] == S_TYPE_OP_CODE)
                op2 <= s_type_immediate;    
            else
                op2 <= instruction_ext;
        else   
            op2 <= ReadData2;
                      
    // Instance the ALU module  
    alu alu0 (.op1(ReadData1), .op2(op2), .alu_op(ALUCtrl), .zero(Zero), .result(ALUresult));

    assign dWriteData = ReadData2;
    assign dAddress = ALUresult;
    
    // For the data block, assigns the writebackdata to the alu result or the read data if memtoreg is high.
    always_comb
        if(MemtoReg)
            WriteBackData <= dReadData;
        else
            WriteBackData <= ALUresult;

endmodule