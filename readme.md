# Before you start...

- when working with the motor, try not to touch it since it could hurts you
- make sure the wire connection is correct, check the wiring part below
- make sure after compilation there's no errors before you push the code to dev board
- there're two keys on the dev board, one is reset and one is user key. I don't really remember so you will have to try which is which. (No big deal if you press the reset key, it doesn't erase the program but instead restart the program)
- the code has not being fully tested, so there might be potential bugs. But most likely it's your code that raise the errors

# overview

This is the first HW to get you guys familiar with coding using CubeMX and Keil. You don't have to understand how the motor communication works since it is too advance for now. The goal is to spin the motor while the user key on the dev board is pressed, and stop the motor when it is released.

## steps

Enable user_key, which is on PA0. Set it the same way you set LED, but this time as "GPIO_input"

Check the motor id: the green light will blink several times within a second on the ESC. The times it blinks is its id (e.g. blink twice a second means id is 2)

Change the macro **ROBOT_ID** to corresponding id

Called the turnOnMotor() function to turn on the motor and turnOffMotor() to turn it off. Remark: you have to continuously calls the function in order for the PID to work. If you don't understand what I mean here, ask team leads.

## wiring

the dev board is connected to the ESC (electronic speed controller) through 2 pins can wire, the motor is connected to the ESC, and battery is connected to both dev board and ESC (ask team leads for help if you couldn't understand this)
