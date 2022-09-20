// This timescale statement indicates that each time tick of the simulator
// is 1 nanosecond and the simulator has a precision of 1 picosecond. This 
// is used for simulation and all of your SystemVerilog files should have 
// this statement at the top. 
`timescale 1 ns / 1 ps 

/***************************************************************************
* 
* File: UpDownButtonCount.sv
*
* Author: Noah Hanks
* Class: ECEN 323, Winter Semester 2022
* Date: 1/5/2022
*
* Module: UpDownButtonCount
*
* Description:
*    This module is able to count up or down as well as reset the cound using the 
*	buttons on the board. The up and down buttons increment and decrement by one respectively
*	while the left and right decrement and increment respectively by the value on the switches.
*	
*
****************************************************************************/

module UpDownButtonCount(clk, btnc, btnu, btnd, btnl, btnr, led, sw);

	input wire logic clk, btnc, btnu, btnd, btnl, btnr;
	input wire logic [15:0] sw;
	output logic [15:0] led;
	
	// The internal 16-bit count signal. 
	logic [15:0] count_i;
	// The increment counter output from the one shot module
	logic inc_count0, inc_count1, inc_count2, inc_count3;
	// reset signal
	logic rst;
	// increment signals (synchronized version of buttons)
	logic btnu_d, inc0;
	logic btnd_d, inc1;
	logic btnl_d, inc2;
	logic btnr_d, inc3;

	// Assign the 'rst' signal to button c
	assign rst = btnc;

	// Create a synchonizer for buttons (synchronize the button to the clock)
	always_ff@(posedge clk)
		if (rst) begin
			btnu_d <= 0;
			inc0 <= 0;
			btnd_d <= 0;
			inc1 <= 0;
			btnl_d <= 0;
			inc2 <= 0;
			btnr_d <= 0;
			inc3 <= 0;
		end
		else begin
			btnu_d <= btnu;
			inc0 <= btnu_d;     
			btnd_d <= btnu;
			inc1 <= btnd_d;
			btnl_d <= btnu;
			inc2 <= btnl_d;
			btnr_d <= btnu;
			inc3 <= btnr_d;       
		end

	// Instance the OneShot module
	OneShot os0 (.clk(clk), .rst(rst), .in(inc0), .os(inc_count0));
	OneShot os1 (.clk(clk), .rst(rst), .in(btnd), .os(inc_count1));
	OneShot os2 (.clk(clk), .rst(rst), .in(btnl), .os(inc_count2));
	OneShot os3 (.clk(clk), .rst(rst), .in(btnr), .os(inc_count3));

	// 16-bit Counter. Increments for btnu and decrements for btnd
	// btnr increments by the value of the switches while brnl will decrement the same value.
	//
	// This is an exmaple of a 'sequential' statement that will synthesize flip-flops
	// as well as the logic for incrementing the count value.
	//
	//  CODING STANDARD: Every "segment/block" of your RTL code must have at least
	//  one line of white space between it and the previous and following block. Also,
	//  ALL always blocks must have a coment.
	always_ff@(posedge clk)
		if (rst)
			count_i <= 0;
		else if (inc_count0)
			count_i <= count_i + 1;
		else if (inc_count1)
			count_i <= count_i - 1;
		else if (inc_count2)
			count_i <= count_i - sw;
		else if (inc_count3)
			count_i <= count_i + sw;
		
	
	// Assign the 'led' output the value of the internal count_i signal.
	assign led = count_i;

endmodule
