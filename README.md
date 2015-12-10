#Micrium OS projects

This directory contains projects developped during the lab sessions on Freescale FRDM K64-F board, using the real-time kernel Micrium.
Note that in this repository are contained only those source codes that have been modifyed with respect to an original folder in the 
local repository.

Projects are the following:

- blinking RGB:
two tasks, one for blinking the RED LED with frequency of 1 Hz, and one for blinking the GREEN LED with frequency of 2 Hz.

- custom GPIO for HC-SR 04:
Customize the board in order to add two additional GPIOs for handling the distance sensor.

- interrupt switch RGB:
Create a simple application where an interrupt is used to turn ON/OFF the BLUE LED every time the SW2 button is pressed.

- semaphore signaling:
Create two tasks, one to turn on/off RED LED each time SW2 is pressed and released, and one to turn on/off GREEN LED each time SW3 
is pressed and released. The two tasks operate as follows:
• Each time SW2 is pressed and released, RED LED is turned on if off, otherwise it is
turned off if already on;
• Each time SW3 is pressed and released, GREEN LED is turned on if off, otherwise it is
turned off if already on;
In order to avoid busy-form-of-waiting, SW2 and SW3 are handled through interrupts and the tasks must be blocked in case the 
trigger event (i.e., the button is pressed and released) is not received.

- switch_RGB:
Create two tasks, one to control the SW2 and the RED LED, and one for controlling the SW3 and the GREEN LED. The two tasks operate 
as follows:
• As long as SW2 is pressed, RED LED is turned on; it is turned off otherwise;
• As long as SW3 is pressed, GREEN LED is turned on; it is turned off otherwise.
The two tasks operate in mutual exclusion: if SW2 is pressed, SW3 is ignored, and vice-versa.
A semaphore shall be used to guarantee the mutual exclusion
