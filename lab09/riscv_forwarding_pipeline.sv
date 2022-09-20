// This timescale statement indicates that each time tick of the simulator
// is 1 nanosecond and the simulator has a precision of 1 picosecond. This 
// is used for simulation and all of your SystemVerilog files should have 
// this statement at the top. 
`timescale 1 ns / 1 ps 
`include "riscv_datapath_constants.sv"
`include "riscv_alu_constants.sv"

/***************************************************************************
* 
* File: riscv_forwarding_pipeline.sv
*
* Author: Noah Hanks
* Class: ECEN 323, Winter Semester 2022
* Date: 3-22-2022
*
* Module: riscv_forwarding_pipeline
*
* Description:
*    This module implements the controller for a simple riscv processor with forwarding
*
****************************************************************************/

module riscv_forwarding_pipeline(clk, rst, instruction, ALUResult, dReadData, PC, dAddress,
    dWriteData, MemRead, MemWrite, WriteBackData, iMemRead);
        
    parameter INITIAL_PC = 32'h00400000;

    input wire logic clk;
    input wire logic rst;
    input wire logic [31:0] instruction;
    input wire logic [31:0] dReadData;
    
    output logic [31:0] PC;
    output logic [31:0] ALUResult;
    output logic [31:0] dAddress;
    output logic [31:0] dWriteData;
    output logic [31:0] WriteBackData;
    output logic MemRead;
    output logic MemWrite;
    output logic iMemRead;
    
    logic Zero;

    logic [4:0] readReg1 = 0;
    logic [4:0] readReg2 = 0;
    logic [4:0] write_Reg = 0;
    
    //////////////////////////////////////////////////////////////////////
    // Pipeline Registers
    /////////////////////////////////////////////////////////////////////
    
    logic [31:0] if_PC = 0;
    
    logic [31:0] id_immediateData = 0;
    logic [31:0] id_PC = 0;
    logic id_ALUSrc = 0;
    logic [3:0] id_ALUCtrl = 0;
    logic id_RegWrite = 0;
    logic id_MemToReg = 0;
    logic id_MemWrite = 0;
    logic id_MemRead = 0;
    logic [31:0] id_op2 = 0;   
    logic [4:0] id_writeReg = 0;
    logic [31:0] id_readData1;
    logic [31:0] id_readData2;
    logic id_Branch = 0;
    logic[4:0] id_rs1 = 0;
    logic[4:0] id_rs2 = 0;
    logic id_isImmed = 0;
    
    logic [31:0] ex_PC = 0;
    logic [31:0] ex_immediateData = 0;
    logic ex_RegWrite = 0;
    logic ex_MemToReg = 0;
    logic ex_MemWrite = 0;
    logic ex_MemRead = 0;  
    logic [4:0] ex_writeReg = 0;
    logic [31:0] ex_readData2 = 0;
    logic [31:0] ex_ALUResult = 0;
    logic ex_Branch = 0;
    logic ex_Zero = 0;
    
    logic mem_PCSrc = 0;
    logic mem_RegWrite = 0;
    logic [4:0] mem_writeReg = 0;
    logic mem_MemToReg = 0;
    logic [31:0] mem_ReadData = 0;
    logic [31:0] mem_branchTarget = 0;
    logic [31:0] mem_ALUResult = 0;
    
    logic wb_regWrite = 0;
    logic [4:0] wb_writeReg = 0;
    logic wb_PCSrc = 0;
    
    //////////////////////////////////////////////////////////////////////
    // HAZARD DETECTION
    /////////////////////////////////////////////////////////////////////
    logic load_use_hazard = 0;
    logic branch_mem_taken = 0;
    logic branch_wb_taken = 0;
    
    always_comb begin
        if(id_MemRead && (id_writeReg == readReg1 || id_writeReg == readReg2)) begin
            load_use_hazard = 1'b1;
            iMemRead = 0;
        end
        else begin
            load_use_hazard = 1'b0;
            iMemRead = 1;
        end
            
        if(ex_Branch && ex_Zero)
            branch_mem_taken = 1'b1;
        else
            branch_mem_taken = 1'b0;
            
        if(wb_PCSrc)
            branch_wb_taken = 1'b1;
        else
            branch_wb_taken = 1'b0;
            
        if(load_use_hazard && (branch_mem_taken || branch_wb_taken)) begin
            load_use_hazard = 1'b0;
            iMemRead = 1;
        end
    end
    
    //////////////////////////////////////////////////////////////////////
    // IF: Instruction Fetch
    /////////////////////////////////////////////////////////////////////	
    
    //Combinational update of top level PC
    always_comb begin
        PC = if_PC;
    end
    
    //Logic for Program Counter to do reset, + 4, and + offset
    always_ff@(posedge clk) begin
        if(rst) begin
            if_PC <= INITIAL_PC;
        end
        else if(mem_PCSrc) begin
            if_PC <= mem_branchTarget;
        end
        else if(!load_use_hazard)begin
            if_PC <= PC + SINGLE_INSTRUCTION_OFFSET;
        end
    end    
        
    //////////////////////////////////////////////////////////////////////
    // ID: Instruction Decode
    /////////////////////////////////////////////////////////////////////
    
    //Local signals
    logic ALUSrc = 0;
    logic [31:0] immediateData = 0;    
    logic [3:0] ALUCtrl = 0;
    logic RegWrite = 0;
    logic MemToReg = 0;
    logic MemWriteLocal = 0;    //Local meaning used only in ID stage to calculate what value the pipeline register should get
    logic MemReadLocal = 0;
    logic Branch = 0;
    logic isImmediate = 0;
    
    //Combination logic for ALUSrc
    always_comb begin
        if(instruction[6:0] == LOAD_OP || instruction[6:0] == S_TYPE_OP
            || instruction[6:0] == I_TYPE_OP)
            ALUSrc = 1'b1;
        else
            ALUSrc = 1'b0;
    end   
    
    //Combination logic for ALUCtrl
    always_comb begin
        if(instruction[6:0] == LOAD_OP || instruction[6:0] == S_TYPE_OP)
            ALUCtrl = ALUOP_ADD;
        else if(instruction[6:0] == SB_TYPE_OP)
            ALUCtrl = ALUOP_SUB;                
        else if(instruction[6:0] == I_TYPE_OP || instruction[6:0] == R_TYPE_OP)
            if(instruction[14:12] == ADD_SUB_F3) begin
                if(instruction[6:0] == R_TYPE_OP && instruction[31:25] == SUB_F7)
                    ALUCtrl = ALUOP_SUB;
                else
                    ALUCtrl = ALUOP_ADD;
            end
            else if(instruction[14:12] == SLT_F3)
                ALUCtrl = ALUOP_LESS;
            else if(instruction[14:12] == AND_F3)
                ALUCtrl = ALUOP_AND;
            else if(instruction[14:12] == OR_F3)
                ALUCtrl = ALUOP_OR;
            else if(instruction[14:12] == XOR_F3)
                ALUCtrl = ALUOP_XOR;
            else
                ALUCtrl = ALUOP_AND;
        else
            ALUCtrl = 4'bxxxx;
    end
    
    //Combination logic for MemRead
    always_comb begin
        if(instruction[6:0] == LOAD_OP)
            MemReadLocal = 1'b1;
        else
            MemReadLocal = 1'b0;
    end
    
    //Combination logic for MemWrite
    always_comb begin
        if(instruction[6:0] == S_TYPE_OP)
            MemWriteLocal = 1'b1;
        else
            MemWriteLocal = 1'b0;
    end
    
    //Combination logic for MemToReg
    always_comb begin
        if(instruction[6:0] == LOAD_OP)
            MemToReg = 1'b1;
        else
            MemToReg = 1'b0;
    end
    
    //Combination logic for RegWrite
    always_comb begin
        if(instruction[6:0] == LOAD_OP || instruction[6:0] == I_TYPE_OP || instruction[6:0] == R_TYPE_OP)
            RegWrite = 1'b1;
        else
            RegWrite = 1'b0;
    end  
    
    //Instantiate register file
    regfile myRegister(.clk(clk), .readReg1(readReg1), .readReg2(readReg2),
	   .writeReg(wb_writeReg), .writeData(WriteBackData), .write(wb_regWrite),
	   .readData1(id_readData1), .readData2(id_readData2));
	  
	//Instruction decoding logic 
    always_comb begin
       readReg1 = 0;
       readReg2 = 0;
       write_Reg = 0;
       immediateData = 0;
       isImmediate = 0;
       
       if(instruction[6:0] == SB_TYPE_OP)
           Branch = 1'b1;
       else
           Branch = 1'b0;
	
	   if(instruction[6:0] == I_TYPE_OP || instruction[6:0] == LOAD_OP) begin
	       immediateData = {{20{instruction[31]}}, instruction[31:20] };
	       readReg1 = instruction[19:15];
	       write_Reg = instruction[11:7];
	       isImmediate = 1'b1;
	   end
	   else if(instruction[6:0] == R_TYPE_OP) begin
	       readReg1 = instruction[19:15];
	       readReg2 = instruction[24:20];
	       write_Reg = instruction[11:7];
	       isImmediate = 1'b0;
	   end
	   else if(instruction[6:0] == S_TYPE_OP) begin
	       readReg1 = instruction[19:15];
	       readReg2 = instruction[24:20];
	       immediateData = {{20{instruction[31]}}, instruction[31:25], instruction[11:7] };
	       isImmediate = 1'b1;
	   end
	   else if(instruction[6:0] == SB_TYPE_OP) begin
	       readReg1 = instruction[19:15];
	       readReg2 = instruction[24:20];
	       immediateData = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0 };
	       isImmediate = 1'b0;
	   end
	end
	
	//Synchronously write data to next pipeline register
	always_ff@(posedge clk) begin
	   if(rst) begin
           id_PC <= INITIAL_PC;
           id_ALUSrc <= 0;
           id_immediateData <= 0;
           id_ALUCtrl <= 0;
           id_RegWrite <= 0;
           id_MemToReg <= 0;
           id_MemWrite <= 0;
           id_MemRead <= 0;
           id_writeReg <= 0;
           id_Branch <= 0;
           id_rs1 <= 0;
           id_rs2 <= 0;
           id_isImmed <= 0;
       end
       else begin
           if(!load_use_hazard)
	           id_PC <= if_PC;
           id_ALUSrc <= ALUSrc;
           id_immediateData <= immediateData;
           id_ALUCtrl <= ALUCtrl;
           if(!load_use_hazard && !branch_mem_taken && !branch_wb_taken) begin
               id_RegWrite <= RegWrite;
               id_MemToReg <= MemToReg;
               id_MemWrite <= MemWriteLocal;
               id_MemRead <= MemReadLocal;
               id_rs1 <= readReg1;
               id_rs2 <= readReg2;
               id_writeReg <= write_Reg;
               id_Branch <= Branch; 
           end
           else begin
               id_RegWrite <= 0;
               id_MemToReg <= 0;
               id_MemWrite <= 0;
               id_MemRead <= 0;
               id_rs1 <= 0;
               id_rs2 <= 0;
               id_writeReg <= 0;
               id_Branch <= 0; 
           end          
           id_isImmed <= isImmediate;
       end
	end
    
    //////////////////////////////////////////////////////////////////////
    // EX: Execute
    /////////////////////////////////////////////////////////////////////
    
    logic [31:0] op1 = 0;
    logic [31:0] op2 = 0;
    logic[31:0] forwardedValue = 0;
    
    //Forwarding muxes
    always_comb begin
        if(id_rs1 == ex_writeReg && ex_RegWrite && ex_writeReg != 0)
            op1 = ex_ALUResult;
        else if(id_rs1 == mem_writeReg && mem_RegWrite && !mem_MemToReg && mem_writeReg != 0)
            op1 = mem_ALUResult;
        else if(id_rs1 == mem_writeReg && mem_RegWrite && mem_MemToReg && mem_writeReg != 0)
            op1 = dReadData;
        else
            op1 = id_readData1;
        
        if(id_rs2 == ex_writeReg && ex_RegWrite && ex_writeReg != 0)
            forwardedValue = ex_ALUResult;
        else if(id_rs2 == mem_writeReg && mem_RegWrite && !mem_MemToReg && mem_writeReg != 0)
            forwardedValue = mem_ALUResult;
        else if(id_rs2 == mem_writeReg && mem_RegWrite && mem_MemToReg && mem_writeReg != 0)
            forwardedValue = dReadData;
        else
            forwardedValue = id_readData2;
           
        if(id_isImmed)
            op2 = id_immediateData;
        else 
            op2 = forwardedValue;
    end
    
    //Instantiate ALU module
	alu myAlu(.op1(op1), .op2(op2), .alu_op(id_ALUCtrl), .zero(Zero), .result(ALUResult));
	
	//Synchronously write data to next pipeline register
	always_ff@(posedge clk) begin
	   if(rst) begin
           ex_PC <= INITIAL_PC;
           ex_RegWrite <= 0;
           ex_MemToReg <= 0;
           ex_MemWrite <= 0;
           ex_MemRead <= 0;  
           ex_Branch <= 0;
           ex_Zero <= 0;
           ex_immediateData <= 0;
       end
       else begin
	       ex_PC <= id_PC;
	       if(!branch_wb_taken && !branch_mem_taken) begin
               ex_RegWrite <= id_RegWrite;
               ex_MemToReg <= id_MemToReg;
               ex_MemWrite <= id_MemWrite;
               ex_MemRead <= id_MemRead;
               ex_ALUResult <= ALUResult;
               ex_Branch <= id_Branch;
               ex_writeReg <= id_writeReg;
           end
           else begin
               ex_RegWrite <= 0;
               ex_MemToReg <= 0;
               ex_MemWrite <= 0;
               ex_MemRead <= 0;
               ex_ALUResult <= 0;
               ex_Branch <= 0;
               ex_writeReg <= 0;
           end
           ex_readData2 <= forwardedValue;
           ex_Zero <= Zero;
           ex_immediateData <= id_immediateData;
       end
	end	
    
    //////////////////////////////////////////////////////////////////////
    // MEM: Read/Write memory
    /////////////////////////////////////////////////////////////////////
    
    //Set signals for mem read/write
    always_comb begin
        dAddress = ex_ALUResult;
        MemRead = ex_MemRead;
        MemWrite = ex_MemWrite;
        dWriteData = ex_readData2;
        mem_PCSrc = ex_Zero & ex_Branch;
        mem_branchTarget = ex_PC + ex_immediateData - SINGLE_INSTRUCTION_OFFSET;
    end
    
    //Synchronously write data to next pipeline register
	always_ff@(posedge clk) begin
       if(rst) begin
           mem_RegWrite <= 0;
           mem_MemToReg <= 0; 
       end
       else begin
           mem_RegWrite <= ex_RegWrite;
           mem_MemToReg <= ex_MemToReg;  
           mem_writeReg <= ex_writeReg;
           mem_ALUResult <= ex_ALUResult;
       end
       wb_PCSrc <= mem_PCSrc;
	end	
    
    //////////////////////////////////////////////////////////////////////
    // WB: Write back
    /////////////////////////////////////////////////////////////////////
    
    //Set write signals
    always_comb begin
       if(mem_MemToReg)
           WriteBackData = dReadData;
       else
           WriteBackData = mem_ALUResult;
       wb_regWrite = mem_RegWrite;
       wb_writeReg = mem_writeReg;
    end 
    	
endmodule
