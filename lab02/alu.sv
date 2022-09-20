// This timescale statement indicates that each time tick of the simulator
// is 1 nanosecond and the simulator has a precision of 1 picosecond. This 
// is used for simulation and all of your SystemVerilog files should have 
// this statement at the top. 
`timescale 1 ns / 1 ps 
`include "riscv_alu_constants.sv"
/***************************************************************************
* 
* File: alu.sv
*
* Author: Noah Hanks
* Class: ECEN 323, Winter Semester 2022
* Date: 1/11/2022
*
* Module: alu
*
* Description: This module takes a 4 bit input and two 32 bit parameters
*    then does a different operation on the two inputs and outputs a 32 bit result.
*	
****************************************************************************/
module alu(op1, op2, alu_op, zero, result);

    input wire logic [31:0] op1, op2;
    input wire logic [3:0] alu_op;
    output logic zero;
    output logic [31:0] result;

    assign zero = (result == 0);
    
    // This block picks which operation to do based on the 4 bit input.
    always_comb
        case(alu_op)
            ALUOP_AND: result = op1 & op2;
            ALUOP_OR: result = op1 | op2;
            ALUOP_ADD: result = op1 + op2;
            ALUOP_SUB: result = op1 - op2;
            ALUOP_LESS: result = $signed(op1) < $signed(op2);
            ALUOP_SRL: result = op1 >> op2[4:0];
            ALUOP_SLL: result = op1 << op2[4:0];
            ALUOP_SRA: result = $unsigned($signed(op1) >>> op2[4:0]);
            ALUOP_XOR: result = op1 ^ op2; 
            default: result = op1 + op2;
        
        endcase
    



endmodule
