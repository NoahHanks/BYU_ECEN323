restart

# adds the first two 32 bit operands
add_force op1 11110011001000010010111100110111
add_force op2 01100010000111000011111011100111
run 20ns

# does the operation for and
add_force alu_op 0000
run 20ns

# does the operation for or
add_force alu_op 0001
run 20ns

# does the operation for add
add_force alu_op 0010
run 20ns

# does the operation for default
add_force alu_op 0011
run 20ns

# does the operation for sub
add_force alu_op 0110
run 20ns

# does the operation for less than
add_force alu_op 0111
run 20ns

# does the operation for shift right logical
add_force alu_op 1000
run 20ns

# does the operation for shift left logical
add_force alu_op 1001
run 20ns

# does the operation for shift right arithmetic
add_force alu_op 1010
run 20ns

# does the operation for xor
add_force alu_op 1101
run 20ns