# Build Instructions

mkdir build

cd build

qmake -o Makefile ../LegControl/LegControl.pro

make

To run the leg control GUI, run the following under build folder

./LegControl

# Control Mode Selection
The control mode has options of 0 and 1.

When control mode = 0, each leg could be controlled independently. For example, you could performe motor calibration and impedance control for each leg. You need to use `Send` button in order to send the command.

When control mode = 1, the stand up impedance control will be active immediately. The `Send` button would not be functioning in this mode. However, you could adjust the height of the robot, and disable/enable robot legs, which are not controlled by `Send`.
