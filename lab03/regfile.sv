`timescale 1ns / 1ps
/***************************************************************************
* 
* File: regfile.sv
*
* Author: Noah Hanks
* Class: ECEN 323, Winter Semester 2022
* Date: 1/18/2022
*
* Module: regfile
*
* Description: This module is able to read and write to the various registers. It also makes sure
*    that the 0 register is not written to but is always zero.
*	
****************************************************************************/



module regfile(clk, readReg1, readReg2, writeReg, writeData, write, readData1, readData2);

    input wire logic clk;
    input wire logic [4:0] readReg1, readReg2, writeReg;
    input wire logic [31:0] writeData;
    input wire logic write;
    
    output logic [31:0] readData1, readData2;
            
    // Declare multi-dimensional logic array (32 words, 32 bits each)
    logic [31:0] register[31:0];
    // Initialize the eight words
    integer i;
    initial
      for (i=0;i<32;i=i+1)
        register[i] = 0;
        
    //Reads to the given registers then writes to the disired one.    
    always_ff@(posedge clk) begin
        readData1 <= register[readReg1];
        readData2 <= register[readReg2];
        if (write && (writeReg!=0)) begin
          register[writeReg] <= writeData;
          if (readReg1 == writeReg)
              readData1 <= writeData;
          if (readReg2 == writeReg)
              readData2 <= writeData;
       end
     end
    
    
endmodule
