restart
run 20ns

add_force clk {0} {1 5} -repeat_every 10

# runs a little bit for good measure.
run 20ns

#adds the inputs or register 1 and 2 then writes to register 2 and enables write
add_force readReg1 00001 
add_force readReg2 00010 
add_force writeReg 00010
add_force write 1
run 20ns

#adds the write data of some random hex
add_force writeData 09090909 -radix hex
run 20ns

#Reads address 0 and 2 and tries to write to 0.
add_force readReg1 00000 
add_force readReg2 00010 
add_force writeReg 00000
add_force write 1
run 20ns

add_force writeData 8593a60b -radix hex
run 20ns

#Reads address 7 and 15 then writes to 7
add_force readReg1 00111 
add_force readReg2 01111 
add_force writeReg 00111
add_force write 1
run 20ns