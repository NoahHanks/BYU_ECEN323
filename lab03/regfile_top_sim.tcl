restart
run 20ns

add_force clk {0} {1 5} -repeat_every 10

# runs a little bit
run 20ns
# does a couple resets for good measure.
add_force btnu 1
run 20ns
add_force btnu 0
run 20ns
## loads register 1
add_force sw 0400 -radix hex
add_force btnl 1
add_force btnc 0
run 20ns
## loads 1234 into the register
add_force sw 9234 -radix hex
add_force btnl 0
add_force btnc 1
run 20ns
## loads register 2
add_force sw 0800 -radix hex
add_force btnl 1
add_force btnc 0
run 20ns
## loads 6789 into the register
add_force sw b678 -radix hex
add_force btnl 0
add_force btnc 1
run 20ns
## loads register 3
add_force sw 0c41 -radix hex
add_force btnl 1
add_force btnc 0
run 20ns
# does an add function
add_force sw 0002 -radix hex
add_force btnl 0
add_force btnc 1
run 60ns









