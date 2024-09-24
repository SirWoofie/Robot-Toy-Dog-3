///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//EEL4742L Final Project Code: Robot Toy Dog
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Per the datasheet,
//MG90S requires a 50Hz PWM signal.
//For 0 degrees, pulse period must be 1.5ms.
//For 90 degrees, pulse period must be 2ms.
//For -90 degrees, pulse period must be 1ms.

//From testing (arm facing you and pointing down when cables come out the left),
//16 for -75 degrees
//40 for 0 degrees
//72 for 90 degrees
//Beyond 16 <= x <= 72, values are not accepted.

#include "msp.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Global macros
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define PERIOD 640 //50Hz signal | PWM period of 640 is 50Hz. So 50*640=32000 = 1Hz
//#define LED_PERIOD 80 //400Hz signal

#define MAX 72 //Maximum motor rotation
#define MIN 16 //Minimum motor rotation

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Global motor movement array
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t com[8][8] = {
                 {60, 30}, {55, 35}, {50, 40}, {45, 45}, {40, 50}, {35, 55}, {30, 60}, {25, 65}, /*{20, 70}*/
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Functions to control each motor simply by calling
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void motor1(int angle); //leg
void motor2(int angle); //leg
void motor3(int angle); //leg
void motor4(int angle); //leg
void motor5(int angle); //tail

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////Functions to control ARGB LEDs (ARGB LEDs are ultimately unsuccessful and disabled)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void rgb1(int value);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Delay in milliseconds
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void delayMs(int n) {
    int i, j;
    for (j=0; j < n; j++) {
        for (i = 3000; i > 0; i--) {}
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Functions for movement
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Walk forward
void walkForward();

//Walk backward
void walkBackward();

//Stand
void stand();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// I2C LCD Library for MSP432 created on October 11, 2018 by Hunter H. Code sourced from library for this
//  project on December 3, 2023. Example code from library is used.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//I2C LCD Includes
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "driverlib.h"
#include "i2c_lcd.h"
#include "usb.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//I2C Defines
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define CHILD_ADDRESS   0x3F
#define HAPPYFACE_ADDR  1
#define HEART_ADDR      2
#define DUCK_ADDR       3
#define ENTER_KEY       13
#define BACK_KEY        8

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//I2C-Specific Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void createCustomChars(void);
void UART0_init();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //I2C Configuration
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize LCD with child address
    LCD_init(CHILD_ADDRESS);

    // Add custom chars to CGRAM
    createCustomChars();
	
	//Show that the robot is starting up on the LCD when the program is running
    LCD_clear();
    LCD_writeString("Launching!", 10);
    delayMs(100);
    LCD_clear();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Ports Configuration
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Timer_A0 ports: P2.7 for A0.4 | P2.6 for A0.3 | P2.5 for A0.2 | P2.4 for A0.1 | P2.3 for A0.0 | 11111000 = 0xF8
    P2SEL0 |= 0xF8;
    P2SEL1 &= 0xF8;
    P2DIR |= 0xF8;

    //Switch port: P4.2 and P4.3 | 00001100 = 0x0C
    P4SEL0 &= ~0x0C; //GPIO
    P4SEL1 &= ~0x0C;
    P4DIR &= ~0x0C; //Input
    P4REN |= 0x0C; //Resistor enabled
    P4OUT |= 0x0C; //Pull-up resistor enabled

	//This section was used during testing to debug the I2C bus intermittance issue. Ultimately, the issue was solved by using external 10k-ohm pullup resistors on the SCL and SDA lines instead of using the MSP432's internal pullups.
//    //I2C ports: P1.6 for SDA | P1.7 for SCL | 1100 0000 = 0xC0
//    P1SEL0 |= 0xC0; //Alternative mode 1
//    P1SEL1 &= ~0xC0;
//    P1DIR |= 0xC0; //Output
//    P1REN |= 0xC0; //Resistor enabled
//    P1OUT |= 0xC0; //Pull-up enabled


    //Timer_A2 ports: P5.7 for A2.2 | 10000000 = 0x840
    P5SEL0 |= 0x80;
    P5SEL1 &= 0x80;
    P5DIR |= 0x80;

	//ARGB LEDs are ultimately unsuccessful and disabled
//    //Timer_A1 ports: P7.4 for A1.4 | P7.5 for A1.3 | P7.6 for A1.2 | P7.7 for A1.1 | 11110000 = 0xF0
//    P7SEL0 |= 0xF0;
//    P7SEL1 &= ~0xF0;
//    P7DIR |= 0xF0;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Timer Configuration
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Configure Timer_A0 for the legs
    TIMER_A0->CCR[0] = PERIOD -1; /* PWM Period */
    //Timer_A0.4
    TIMER_A0->CCR[4] = 1; /* begin from 1% */
    TIMER_A0->CCTL[4] = 0xE0; /* CCR4 reset/set mode */
    //Timer_A0.3
    TIMER_A0->CCR[3] = 1; /* begin from 1% */
    TIMER_A0->CCTL[3] = 0xE0; /* CCR3 reset/set mode */
    //Timer_A0.2
    TIMER_A0->CCR[2] = 1; /* begin from 1% */
    TIMER_A0->CCTL[2] = 0xE0; /* CCR2 reset/set mode */
    //Timer_A0.1
    TIMER_A0->CCR[1] = 1; /* begin from 1% */
    TIMER_A0->CCTL[1] = 0xE0; /* CCR1 reset/set mode */
    //End configuration of Timer_A0
    TIMER_A0->CTL = 0x0114; /* use ACLK, count up, clear TA0R register */

    //Configure Timer_A2 for the tail
    TIMER_A2->CCR[0] = PERIOD -1; /* PWM Period */
    //Timer_A2.1
    TIMER_A2->CCR[2] = 1; /* begin from 1% */
    TIMER_A2->CCTL[2] = 0xE0; /* CCR1 reset/set mode */
    //End configuration of Timer_A2
    TIMER_A2->CTL = 0x0114; /* use ACLK, count up, clear TA2R register */

//    //Configure Timer_A1 for the ARGB LEDs (ARGB LEDs are ultimately unsuccessful and disabled)
//    TIMER_A1->CCR[0] = LED_PERIOD -1; /* PWM Period */
//    //Timer_A1.4
//    TIMER_A1->CCR[4] = 1; /* begin from 1% */
//    TIMER_A1->CCTL[4] = 0xE0; /* CCR4 reset/set mode */
//    //End configuration of Timer_A1
//    TIMER_A1->CTL = 0x0114; /* use ACLK, count up, clear TA1R register */

//notes
//motor range is 72-16 = 56. halfway is 28
//motor1, motor2 use "com1"
//motor3, motor4 use "com2"
/* Motor ranges. Exceeding this range will crash into body. A 2x8 array was created as a global variable to store these values when moving. Additionally, values of x5 were interpolated into the array for finer motor control.
com1    com2
60      30
50      40 standing upright
45      45 center of alignment
40      50
30      60
20      70
 */
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //While loop setup
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//The "forward" variable controls the direction of the robot in the while loop. 1 = move forward, 0 = move backward, 2 = stand still.
    uint8_t forward = 1;
	//The "tail" variable controls whether the tail will move clockwise or counterclockwise in the while loop.
    int tail = 0;
	//The "error" variable controls whether the LCD should display an error message. 0 = no error, 1 = movement direction switches are in wrong position.
    uint8_t error = 0;
	//The "count" variable is used to count each iteration of the while loop. This is used to slow down the tail wagging function, so it is only called every other iteration.
    uint8_t count = 0;

    //Disable LCD cursor and cursor blinking
    LCD_cursorOff();
    LCD_blinkOff();

    while (1) {
        //Reset error warning - this should never carry over between iterations.
        error = 0;
        //Increment counter
        count++;

        //Check forwards/backwards cable
        if ((P4IN & 0x0C) == 0) {
            //Both cables disconnected, stand still
            forward = 2;
        } else if ((P4IN & 0x04) == 0) {
            //One cable connected, walk forward
            forward = 1;
        } else if ((P4IN & 0x02) == 0) {
            //One cable connected, walk backwards
            forward = 0;
        } else {
            //No cables connected, stand still and display error
            forward = 2;
            error = 1;
        }

        //LCD Line 1
        LCD_clear();
        LCD_writeString("Hello, I am Dog", 15);
        LCD_writeChar(HAPPYFACE_ADDR);

        //LCD Line 2
        LCD_setCursorPosition(1, 0);
        if (error) { //Error text is too long, so it will overwrite Line 1 then write Line 2.
            LCD_clear();
            LCD_writeString("Error! Cables", 13);
            LCD_setCursorPosition(1, 0);
            LCD_writeString("are wrong!", 10);
        } else if (forward == 2) {
            LCD_writeString("Standing still", 14);
        } else if (forward) {
            LCD_writeString("Walking forward", 15);
        } else {
            LCD_writeString("Walking backward", 16);
        }

        //Walking
        if  (forward == 2) {
          //Stand still
            stand();
            delayMs(50);
        } else if (forward) {
            walkForward();
            stand();
            delayMs(50);
        } else {
            walkBackward();
            delayMs(50);
        }


        //Tail wagging (but not as frequent as walking)
        if (count >= 2) {
            count = 0;

            tail ^= 1;
            if (tail) {
                motor5(16);
            } else {
                motor5(72);
            }
        }

    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Walk forward function - call this multiple times! Moves the legs counterclockwise then clockwise.
void walkForward() {
    uint8_t i = 0;

    //Move motors 2 and 3
    for (i = 2; i <= 7; i++) {
        motor2(com[i][0]);
        motor3(com[i][1]);
        delayMs(10);
    }
    //Move motors 1 and 4
    for (i = 2; i <= 7; i++) {
        motor1(com[i][0]);
        motor4(com[i][1]);
        delayMs(10);
    }

    //Lastly, return the motors to starting position

    //Move motors 2 and 3
    for (i = 7; i >= 2; i--) {
        motor2(com[i][0]);
        motor3(com[i][1]);
        delayMs(10);
    }
    //Move motors 1 and 4
    for (i = 7; i >= 2; i--) {
        motor1(com[i][0]);
        motor4(com[i][1]);
        delayMs(10);
    }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Walk backward function - call this multiple times! Moves the legs clockwise then counterclockwise.
void walkBackward() {
    int i = 0;

    //Move motors 1 and 4
    for (i = 2; i >= 0; i--) {
        motor1(com[i][0]);
        motor4(com[i][1]);
        delayMs(10);
    }
    //Move motors 2 and 3
    for (i = 2; i >= 0; i--) {
        motor2(com[i][0]);
        motor3(com[i][1]);
        delayMs(10);
    }

    //Lastly, return the motors to starting position
    //Move motors 1 and 4
    for (i = 0; i <= 2; i++) {
        motor1(com[i][0]);
        motor4(com[i][1]);
        delayMs(10);
    }
    //Move motors 2 and 3
    for (i = 0; i <= 2; i++) {
        motor2(com[i][0]);
        motor3(com[i][1]);
        delayMs(10);
    }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Stand still, upright. Returns the motors to a neutral position. If the motors are too far from this position, they may move very abruptly.
void stand() {
    motor1(50);
    motor2(50);
    motor3(40);
    motor4(40);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Functions to control each motor simply by calling. These functions input an "angle" value which is the PWM control for the Timer_A module associated with the motor. All of the complex work is abstracted out so all that must be done is call the function and pass in the PWM frequency.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Motor1 uses P2.7 and T_A0.4
void motor1(int angle) {
    TIMER_A0->CCR[4] = angle;
    if (TIMER_A0->CCR[4] > PERIOD) { /* wrap around when reaches 100% */
        TIMER_A0->CCR[4] = 1; /* begin from 1% */
    }
    /* wait a cycle */
    while((TIMER_A0->CCTL[0]& 1) == 0); /*wait until the CCIFG of CCR0 is set*/
    TIMER_A0->CCTL[0]&= ~1; /* clear the CCIFG flag */
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Motor2 uses P2.6 and T_A0.3
void motor2(int angle) {
    TIMER_A0->CCR[3] = angle;
    if (TIMER_A0->CCR[3] > PERIOD) { /* wrap around when reaches 100% */
        TIMER_A0->CCR[3] = 1; /* begin from 1% */
    }
    /* wait a cycle */
    while((TIMER_A0->CCTL[0]& 1) == 0); /*wait until the CCIFG of CCR0 is set*/
    TIMER_A0->CCTL[0]&= ~1; /* clear the CCIFG flag */
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Motor3 uses P2.5 and T_A0.2
void motor3(int angle) {
    TIMER_A0->CCR[2] = angle;
    if (TIMER_A0->CCR[2] > PERIOD) { /* wrap around when reaches 100% */
        TIMER_A0->CCR[2] = 1; /* begin from 1% */
    }
    /* wait a cycle */
    while((TIMER_A0->CCTL[0]& 1) == 0); /*wait until the CCIFG of CCR0 is set*/
    TIMER_A0->CCTL[0]&= ~1; /* clear the CCIFG flag */
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Motor4 uses P2.4 and T_A0.1
void motor4(int angle) {
    TIMER_A0->CCR[1] = angle;
    if (TIMER_A0->CCR[1] > PERIOD) { /* wrap around when reaches 100% */
        TIMER_A0->CCR[1] = 1; /* begin from 1% */
    }
    /* wait a cycle */
    while((TIMER_A0->CCTL[0]& 1) == 0); /*wait until the CCIFG of CCR0 is set*/
    TIMER_A0->CCTL[0]&= ~1; /* clear the CCIFG flag */
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Motor5 uses P5.7 and T_A2.2
void motor5(int angle) {
    TIMER_A2->CCR[2] = angle;
    if (TIMER_A2->CCR[2] > PERIOD) { /* wrap around when reaches 100% */
        TIMER_A2->CCR[2] = 1; /* begin from 1% */
    }
    /* wait a cycle */
    while((TIMER_A2->CCTL[0]& 1) == 0); /*wait until the CCIFG of CCR0 is set*/
    TIMER_A2->CCTL[0]&= ~1; /* clear the CCIFG flag */
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ARGB Functions (ARGB LEDs are ultimately unsuccessful and disabled)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//RGB1 uses P7.4 and T_A1.4
//void rgb1(int value) {
//    TIMER_A1->CCR[4] = value;
//    if (TIMER_A1->CCR[4] > LED_PERIOD) { /* wrap around when reaches 100% */
//        TIMER_A1->CCR[4] = 1; /* begin from 1% */
//    }
//    /* wait a cycle */
//    while((TIMER_A1->CCTL[0]& 1) == 0); /*wait until the CCIFG of CCR0 is set*/
//    TIMER_A1->CCTL[0]&= ~1; /* clear the CCIFG flag */
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//I2C Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Create the custom characters for the display
void createCustomChars(void)
{
    uint8_t happyFace[CHAR_HEIGHT] = { 0x00, 0x00, 0x0A, 0x00, 0x11, 0x0E, 0x00 };
    uint8_t heart[CHAR_HEIGHT]     = { 0x00, 0x0A, 0x1F, 0x1F, 0x0E, 0x04, 0x00 };
    uint8_t duck[CHAR_HEIGHT]      = { 0x00, 0x0C, 0x1D, 0x0F, 0x0F, 0x06, 0x00 };

    // Store happy face in memory address 0, heart in memory address 1....
    LCD_createChar(HAPPYFACE_ADDR, happyFace);
    LCD_createChar(HEART_ADDR, heart);
    LCD_createChar(DUCK_ADDR, duck);

    // Creating custom char moves the cursor, we'll reset it
    LCD_clear();
    LCD_home();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//UART0 Initialization
void UART0_init() {
   EUSCI_A0->CTLW0 |= 1;
   EUSCI_A0->MCTLW = 0;
   EUSCI_A0->CTLW0 = 0x0081;
   EUSCI_A0->BRW = 26;
   P1SEL0 |= 0x0c;
   P1SEL1 &= ~0x0c;
   EUSCI_A0->CTLW0 &= ~1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//End of program.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
