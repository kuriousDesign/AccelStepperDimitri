# DIMITRI CODE
Here is a list of instructions on how to use this device

## CONTROLLING THE MOTION DEVICE FROM COMPUTER SERIAL USB CONNECTION

1. connect your computer to the mega that has the DimitriMotion.ino file loaded on it
2. open Arduino, select mega as your board and select the com port that is connected to the physical mega
3. open serial monitor and set baud rate to 115200 and use LF as line ending
4. type D0 into the text field and hit ENTER, this will allow the DimitriMotion board to talk to you
5. use the list of commands below to control the board

### STATUS READOUT - gives you the current mode of the board, step of the mode, motor position, current gear and target gear
X0

### MANUAL MODE - enables use of up an down shifters to move (jog) the motor positive and negative direction, respectively
M01100

### HOMING MODE - attempts to home the motor, will either finish in idle mode or will deactive if it fails
M0200

### IDLE MODE - allows you to control the motor as intended for normal use, in order to enter this mode you must initiate homing first (see above)
