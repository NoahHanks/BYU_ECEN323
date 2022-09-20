`timescale 1ns / 1ps
/***************************************************************************
* 
* File: regfile_top.sv
*
* Author: Noah Hanks
* Class: ECEN 323, Winter Semester 2022
* Date: 1/18/2022
*
* Module: regfile_top
*
* Description: Thsi module takes input from the buttons and switches and loads the different registers and values 
*    into those registers. The ALU is also added to perform different operations.
*	
****************************************************************************/


module regfile_top(clk, btnc, btnl, btnu, btnd, sw, led);
    
    input wire logic clk, btnc, btnl, btnu, btnd;
    input wire logic [15:0] sw;
    
    output logic [15:0] led;
    
    logic rst, btncc, zero, regWrite;
    logic [4:0] readReg1, readReg2, writeReg;
    logic [14:0] register;
    logic [31:0] readData1, readData2, regWriteData, result, sw_ext;

    // Assign the 'rst' signal to button c
	assign rst = btnu;
	assign sw_ext = {{17 {sw[14]}}, sw[14:0]};
	
	assign readReg1 = register[4:0];
    assign readReg2 = register[9:5];
    assign writeReg = register[14:10];

	// Does the different operations based on which buttons or switches are pushed.	
    always_ff@(posedge clk) begin
     // Create a synchonizer for buttons (synchronize the button to the clock)
        btncc <= btnc;

//shows the upper or lower parts of readData1 if the down button is pressed
        if(btnd)
            led <= readData1[31:16];
        else
            led <= readData1[15:0];
//reests for btnu or loads the register based on the switches
        if(btnu)
            register <= 0;
        else if(btnl)
            register <= sw[14:0];
//if the leftmost switch is up, the data is the result, if not, it's the sign extension of the switches
        if(!sw[15])
            regWriteData <= result;
        else
            regWriteData <= sw_ext;
    end
      
    // Instance the OneShot module
	OneShot os0 (.clk(clk), .rst(rst), .in(btncc), .os(regWrite));
	
	// Instance the regfile module
    regfile regfile0 (.clk(clk), .readReg1(readReg1), 
        .readReg2(readReg2), .writeReg(writeReg), .writeData(regWriteData), 
        .write(regWrite), .readData1(readData1), .readData2(readData2));
    
    // Instance the ALU module
    alu alu0 (.op1(readData1), .op2(readData2), .alu_op(sw[3:0]), .zero(zero), .result(result));
	
    
endmodule
