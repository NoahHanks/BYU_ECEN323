#######################
# fib_iterative.s
# Name: Noah Hanks
# Date: Jan 25 2022
#
# File to iteratively loop through to find the fibonacci number.
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
	
	beqz a0, return			# Checks if a == 0
	li t1, 1			# Sets temporary register to 1
	beq a0, t1, return		# Checks if a == 1
	li s5, 2			# Sets i = 2
	li s0, 0			# Sets fib2 = 0
	li s1, 1			# Sets fib1 = 1
	li s2, 0			# Sets fib = 0
	j loop				# Starts the loop
	
loop:	
	bgt s5, a0, return		# Returns if i is greater than a
	add s0, s1, s2			# Sets fib = fib1 + fib2
	mv s2, s1			# Sets fib2 = fib1
	mv s1, s0			# Sets fib1 = fib
	addi s5, s5, 1			# Increments i
	j loop				# Jumps back to the loop
	

return:
	mv a0, s0			# Moves the fib number to a0
	ret				# Returns from jap fibonacci
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
