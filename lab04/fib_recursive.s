#######################
# fib_recursive.s
# Name: Noah Hanks
# Date: Jan 25 2022
#
# File to recursively loop through to find the fibonacci number.
#
#######################
.globl  main

.data
fib_input:
	.word 10
	
result_str:
	.string "\nFibinnoci Number is "

.text
main:

	# Load n into a0 as the argument
	lw a0, fib_input
	
	# Call the fibinnoci function
	jal fibinocci
	
	# Save the result into s2
	mv s2, a0 

	# Print the Result string
	la a0,result_str      # Put string pointer in a0
        li a7,4               # System call code for print_str
        ecall                 # Make system call

        # Print the number        
 	mv a0, s2
        li a7,1               # System call code for print_int
        ecall                 # Make system call

	# Exit (93) with code 0
        li a0, 0
        li a7, 93
        ecall
        ebreak

fibinocci:

	# This is where you should create your Fibinnoci function.
	# The input argument arrives in a0. You should put your result
	# in a0.
	#
	# You should properly manage the stack to save registers that
	# you use.
	

	addi sp, sp, -12	# Make room to save values on the stack
	sw s0,  0(sp)		# This function uses 3 callee save regs
	sw s1,  4(sp)		# Each register is 4 bytes
	sw ra, 8(sp)		# The return address needs to be saved  	
        mv s0, a0		# Moves a0 to s0
            
cursed:	    
	beqz s0 uncursed	# Goes to return if s0 == 0
	li t1, 1		# Sets temporary register to 1
	beq s0, t1, uncursed  	# Goes to return if s0 == 1
	

	addi a0, s0, -1		# Sets a0 = s0 - 1
	jal fibinocci		# Jumps back to th start of fibonacci
	mv s1, a0		# Moves a0 to s1
	
	addi a0, s0, -2		# Sets a0 = s0 - 2
	jal fibinocci		# Jumps back to th start of fibonacci
	add a0, a0, s1		# Sets a0 = a0 + s1
	
uncursed:
	
	lw s0,  0(sp)		# Restore any callee saved regs used
	lw s1,  4(sp)		# Each register is 4 bytes
	lw ra,  8(sp)		# Each register is 4 bytes
	addi sp, sp, 12		# Update stack pointer
	ret			# Jump to return address
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
