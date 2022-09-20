`timescale 1ns / 1ps
/***************************************************************************
* 
* File: calc.sv
*
* Author: Noah Hanks
* Class: ECEN 323, Winter Semester 2022
* Date: 1/11/2022
*
* Module: calc
*
* Description: This calculator program is has a variety of functions such as 
*    adding, subtracting, shifting, etc. It takes input from switches and buttons
*	then displays the result on the leds.
*
****************************************************************************/

module calc(clk, btnc, btnl, btnu, btnr, btnd, sw, led);
    
    input wire logic clk, btnc, btnu, btnd, btnl, btnr;
	input wire logic [15:0] sw;
	output logic [15:0] led;
	
	// The internal 16-bit count signal. 
	logic [31:0] accumulator_ext, result, sw_ext;
	logic [15:0] accumulator;
	logic [2:0] alu_from_buttons;
	logic [3:0] alu_op;
	// The increment counter output from the one shot module
	logic btnd_alu;
	// reset signal
	logic rst, zero;
	// variables for the synchonized bntd
	logic btnd_d, inc1;

	
	assign alu_from_buttons = {btnl, btnc, btnr};
	assign led = accumulator;
	assign rst = btnu;
	assign accumulator_ext = {{16 {accumulator[15]}}, accumulator};
	assign sw_ext = {{16 {sw[15]}}, sw};
    
    // Create a synchonizer for buttons (synchronize the button to the clock)
	always_ff@(posedge clk) begin
		if (rst) begin
			btnd_d <= 0;
			inc1 <= 0;
		end
		else begin
			btnd_d <= btnu;
			inc1 <= btnd_d;
			end  
		end

    // Instance the OneShot module
	OneShot os0 (.clk(clk), .rst(rst), .in(btnd), .os(btnd_alu));
		
		// Sets the accumulator to zero for a reset or the last 16 bits of result
		// if button down was pressed.
	always_ff@(posedge clk)
	   	if(btnu)
	   	   accumulator <= 0;
	   	else if(btnd_alu)
	   	   accumulator <= result[15:0];  
	   	   
	// Checks the three buttons and picks the correct alu input accordingly
	always_comb
        case(alu_from_buttons)
            3'b000: alu_op = 4'b0010;
            3'b001: alu_op = 4'b0110;
            3'b010: alu_op = 4'b0000;
            3'b011: alu_op = 4'b0001;
            3'b100: alu_op = 4'b1101;
            3'b101: alu_op = 4'b0111;
            3'b110: alu_op = 4'b1001;
            3'b111: alu_op = 4'b1010;
            endcase
            
	// Instance ALU module
	alu alu0 (.op1(accumulator_ext), .op2(sw_ext), .alu_op(alu_op), .zero(zero), .result(result));
	
	
    
endmodule
