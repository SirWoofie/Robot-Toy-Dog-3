# Robot-Toy-Dog-3
A class project to create a toy. This toy is a robot dog that walks back and forth, has a face, and has a tail. This project was created in a rush - as such, the walking is very rudimentary.

# Hardware
* MSP432P401R
* 5 MG60S servo motors
* I2C LCD 1602
* 5V 3A or better battery pack
* 5V ARGB LEDs for additional flair
* Robot body materials
  * Laser-cut acrylic
  * Adhesive suitable for Acrylic
  * Screws to mount servos
 
# Software
This project was built on the TI MSP432 platform using TI Code Composer Studio. The MSP432 DriverLib was sourced to enable quick development of board features like servos and timers.
Unfortunately, the DriverLib may not be utilized in this project.

The program utilizes the MSP432's built-in Timer_A modules for generating the PWM signals to control the servos. It slightly modifies the PWM signals to make the servos move in a fashion similar to legs walking and a tail wagging.

It also utilizes the code from a Hunter Hedges' I2C LCD Library for MSP432. This code has been modified to support machine-generated messages sent to the I2C LCD, like a happy face and notifications. The library can be found here: https://github.com/hunterhedges-zz/I2cLcd
