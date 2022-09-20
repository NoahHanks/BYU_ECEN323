restart

add_force clk {0} {1 5} -repeat_every 10

# does a couple resets for goog measure.
run 50ns
add_force btnu 1
run 20ns
add_force btnu 0
run 20ns
add_force btnu 1
run 20ns
add_force btnu 0
run 20ns

# does the OR function with 0x0 and 0x1234
add_force sw 0001001000110100
run 20ns
add_force btnl 0
add_force btnc 1
add_force btnr 1
run 20ns
add_force btnd 1
run 20ns
add_force btnd 0
run 20ns

# does the  	AND function with 0x1234 and 0x0ff0
add_force sw 0000111111110000
run 20ns
add_force btnl 0
add_force btnc 1
add_force btnr 0
run 20ns
add_force btnd 1
run 20ns
add_force btnd 0
run 20ns

# does the  	Add function with 0x0230 and 0x324f
add_force sw 0011001001001111
run 20ns
add_force btnl 0
add_force btnc 0
add_force btnr 0
run 20ns
add_force btnd 1
run 20ns
add_force btnd 0
run 20ns

# does the  	Sub function with 0x347f and 0x2d31
add_force sw 0010110100110001
run 20ns
add_force btnl 0
add_force btnc 0
add_force btnr 1
run 20ns
add_force btnd 1
run 20ns
add_force btnd 0
run 20ns

# does the XOR function with 0x001 and 0xffff
add_force sw 1111111111111111
run 20ns
add_force btnl 1
add_force btnc 0
add_force btnr 0
run 20ns
add_force btnd 1
run 20ns
add_force btnd 0
run 20ns

# does the Less Than function with 0xf8b1 and 0x7346
add_force sw 0111001101000110
run 20ns
add_force btnl 1
add_force btnc 0
add_force btnr 1
run 20ns
add_force btnd 1
run 20ns
add_force btnd 0
run 20ns

# does the less than function with 0x001 and 0xffff
add_force sw 1111111111111111
run 20ns
add_force btnl 1
add_force btnc 0
add_force btnr 1
run 20ns
add_force btnd 1
run 20ns
add_force btnd 0
run 20ns
