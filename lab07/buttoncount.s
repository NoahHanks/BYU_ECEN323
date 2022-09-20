####################################################################################3#
#
# 	buttoncount.s
#
# This simple test program demonstrates the operation of all the LEDs, switches,
# buttons, and seven segment display in the I/O sub-system. 
#
#  - The timer value is copied to the seven segment display
#  - Button behavior:
#    - BTNC clears the timer/seven segment display
#    - BTND turns all the LEDs OFF
#    - BTNU turns all the LEDs to on
#    - BTNR inverts values from the switches when displaying on LEDs
#    - BTNL shifts the values from the switches left one
#    - No button:
#      The value of the switches are read and then used to drive the LEDs
#
# This version of the program is written using the primitive instruction set
# for the multi-cycle RISC-V processor developed in the first labs.
#
# This program does not use the data segment.
#
# Memory Organization:
#   0x0000-0x1fff : text
#   0x2000-0x3fff : data
#   0x7f00-0x7fff : I/O
#
# Registers:
#  x3(gp):  I/O base address
#  x8(s0):  Value of buttons
#  x9(s1):  Value of switches
#  x18(s2): Value to write in LEDs
#
####################################################################################3#
.globl  main

.data
    .word 0

.text


# I/O address offset constants
    .eqv LED_OFFSET 0x0
    .eqv SWITCH_OFFSET 0x4
    .eqv SEVENSEG_OFFSET 0x18
    .eqv TIMER 0x30
    .eqv BUTTON_OFFSET 0x24

# I/O mask constants
    .eqv BUTTON_C_MASK 0x01
    .eqv BUTTON_L_MASK 0x02
    .eqv BUTTON_D_MASK 0x04
    .eqv BUTTON_R_MASK 0x08
    .eqv BUTTON_U_MASK 0x10

main:
    # Prepare I/O base address
    addi gp, x0, 0x7f
    # Add x3 to itself 8 times (0x7f << 8 = 0x7f00)
    addi t0, x0, 8
L1:
    add gp, gp, gp
    addi t0, t0, -1
    beq t0, x0, L2
    beq x0, x0, L1
L2:
    # 0x7f00 should be in gp (x3)

    # Set constants
    sw x0, SEVENSEG_OFFSET(gp)          # Clear seven segment display
	
LOOP_START:
    # Load the buttons
    lw s0, BUTTON_OFFSET(gp)
    # Read the switches
    # Set switches to the seven segment
    lw s1, SWITCH_OFFSET(gp)
    sw s1, SEVENSEG_OFFSET(gp)

    # Mask the buttons for button C
    andi t0, s0, BUTTON_C_MASK
    # If button is not pressed, skip btnc code
    beq t0, x0, BUTTON_CHECK
    # If button c is pressed, reset the LEDs
	sw, x0,LED_OFFSET(gp)
    # Button C pressed - fall through to clear timer and seven segmeent dislplay
    beq x0, x0, LOOP_START              # Don't process other buttons
       
BUTTON_CHECK:       # Label to check all buttons

BTNU_CHK:
    # Mask the buttons for button U
    andi t0, s0, BUTTON_U_MASK
    # If button is not pressed, skip
    beq t0, t1,BTND_CHK
    add t1, t0, x0 # Update previous button mask
 	beq t0, x0, BTND_CHK
    #I ncrement the LEDs
	lw, s9, LED_OFFSET(gp)
	addi s9, s9, 1
	sw, s9, LED_OFFSET(gp)
	beq x0, x0, LOOP_START

BTND_CHK:           # Check btnd
    # Mask the buttons for button U
    andi t0, s0, BUTTON_D_MASK
    # If button is not pressed, skip
    beq t0, t2, LOOP_START
    add t2, t0, x0 # Update previous button mask
 	beq t0, x0, LOOP_START
    # Decrement the LEDs
	lw, s9, LED_OFFSET(gp)
	addi s9, s9, -1
	sw, s9, LED_OFFSET(gp)
	beq x0, x0, LOOP_START

