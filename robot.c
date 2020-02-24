/*******************************************************************************
* File: RobotIntroCode.C
* By  : chi wen hsuan
* ID : 1301248
*
* Description :
*     Robot calculates its position with constant pwm.
*     Written for IAR compiler, using Normal DLIB Library.
* Limitations :
*     Written for C language (not C++).
*     This program assumes the robot starts at the origin
*     and is pointing along the x axis.
*******************************************************************************/

/*******************************************************************************
* INCLUDES
*******************************************************************************/
#include <iom128.h>
#include <intrinsics.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/*******************************************************************************
* DEFINES
*******************************************************************************/
#define PI 3.141592653589793238462643 // from CLIB math.h

#define RIGHT_SPEED 1023
#define LEFT_SPEED  1023
#define BUF_MAX     1000
#define USE_SERIAL_INTERRUPTS 1

/*******************************************************************************
* STRUCTURES
*******************************************************************************/
struct Position
{
    float x; // x and y position coordinates in metres
    float y;
    float h; // heading angle in radians, anti-clockwise from the x-axis

};

 

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void Setup(void); // ATmega128 initialisation for this program
// calculate robot position from wheel pulse counts
void CalcPosition(struct Position *ppos);
void OutputString(char* str); // put string in serial port 0 transmit buffer
void PID(struct Position *ppos);

/*******************************************************************************
* GLOBAL VARIABLES
*******************************************************************************/
volatile unsigned int leftPulseCount, rightPulseCount; // wheel pulse counts
#if USE_SERIAL_INTERRUPTS == 1
// serial port 0 transmit buffer
char buffer[BUF_MAX]; // buffer (queue) data
volatile unsigned int head, tail, count; // buffer (queue) indexes 
#endif
//Target
float Ty=0;
float Tx=-2;


/*******************************************************************************
* MAIN FUNCTION
*******************************************************************************/
void main (void)
{
    //static char str[40]; // serial output string
    struct Position pos; // current position from CalcPosition
    // set initial robot location at origin
    pos.x = 0.;
    pos.y = 0.;
    pos.h = 0.;
    Setup(); // initialise ATmega128

    enum robotstate {start_state,Turn_state,PID_state,stop_state};
    int robotstate=start_state;
    while (1) // loop forever
    {  
        CalcPosition(&pos); // calculate position from wheel counters 
        
        static float distance=0;
        distance =sqrt(((Tx-pos.x)*(Tx-pos.x))+((Ty-pos.y)*(Ty-pos.y)));
        static float e=0;  
        e=atan2((Ty-(pos.y)),(Tx-(pos.x)))-pos.h;
        if(e >= PI)//if error large than pi
        {
          e = e - 2*PI;
        }
        if(e <= -PI)//if -Pi larger than error 
        {
          e = e + 2*PI;
        }
        switch(robotstate)
      {
        case start_state:
          if(distance<=0.05)
            robotstate=stop_state;
          else
            robotstate=Turn_state;
         break;
        case Turn_state:
          if(distance<=0.05)
            robotstate=stop_state;
         else if(e>0.5)
          {  
            OCR1B=1023;
            OCR1A=0;
          } 
          else if (e<-0.5)
          {
            OCR1A=1023;
            OCR1B =0;
          }
          else
          {
            robotstate=PID_state;
          }
         break;
        case PID_state:
          if(distance<0.05)
          {
            robotstate=stop_state;
          }
          else if(e>0.5||e<-0.5)
          {
            robotstate=Turn_state;
          }
          else
          {  
            PID(&pos);
          }
         break;
        case stop_state:
           OCR1A=0;
           OCR1B=0;
         break;
         
      }
    }
}

/*******************************************************************************
* OTHER FUNCTIONS
*******************************************************************************/
void Setup(void) // ATmega128 setup
{
    TCCR1A=0xA3;// timer1 setup for pwm: enable pwm outputs, mode 7 prescale=256
    TCCR1B=0x0C;
    TCCR1C=0x00;
    //Timer 3  
    TCCR3A=0x80;
    TCCR3B=0x0b;
    TCCR3C=0x80;
    OCR3A=30;
    OCR3B=212;
    // motors off
    OCR1A = 0; 
    OCR1B = 0;
    // enable motor outputs
    DDRA_Bit6 = 1; 
    DDRA_Bit7 = 1;
    // enable motor direction outputs, motors forward
    PORTA_Bit6 = 0; 
    PORTA_Bit7 = 0;
    PORTB_Bit5=1;
    PORTB_Bit6=1;
    DDRB_Bit5=1;
    DDRB_Bit6=1;
    
   
    // enable the wheel pulse generator electronics
    DDRC_Bit3 = 1;
    PORTC_Bit3 = 1;
    
    UCSR0A=0x00;
    UCSR0B_Bit4=1;// serial output: enable receive and transmit
    UCSR0B_Bit3=1;  // 8 data bits, no parity, 1 stop
    UCSR0C_Bit2=1;    // set the baud rate
    UCSR0C_Bit1=1;    // do NOT enable the transmit interrupt here
    UBRR0H =0; // must be first!
    UBRR0L =12;
    
    // give the electronic hardware time to settle
    __delay_cycles(4000000); // 500ms
    // enable external interrupts for the wheel pulses (INT0, INT1)
    EIMSK_Bit0=1;
    EIMSK_Bit1=1;
    EICRA=0x0F;
    #if USE_SERIAL_INTERRUPTS == 1
    // initialise serial output buffer
    head = 0;
    tail = 0;
    count = 0;
    #endif
    __enable_interrupt(); // enable interrupts last
    // display started on serial port
    OutputString("\r\nSTARTING\r\n");
}

void PID(struct Position *ppos)//PID caculation
{
   static float dI,thetaT;
   static float  e=0;
   static float I=0;
   static float D=0;
   static int F=0;
   static float preve = 0;
   thetaT=atan2((Ty-(ppos->y)),(Tx-(ppos->x)));//formula
   e=thetaT-ppos->h;
   dI=(0.1*(e + preve))/2;
   I=I+dI;
   D=(e-preve)/0.1;
   F=(int)(2000*e+100*I+75*D);
   preve = e;
   
    if (F>0)
    {
     OCR1A=1023;
     OCR1B=1023-F;
    }

     else if (F<0)
     {
      OCR1A=1023+F;
      OCR1B=1023;
     }

     else if (F==0)
     {
       OCR1A=1023;
       OCR1B=1023;
     }
}
void CalcPosition(struct Position *ppos) // calculate the robot position
{
    int leftCount, rightCount; // number of wheel pulses
    static char str[100]; // serial output string
    float DTheta,dx, dy;
    // get the pulse counts
    __disable_interrupt();
    leftCount = leftPulseCount; // copy the pulse counts
    rightCount = rightPulseCount;
    leftPulseCount = 0; // reset the pulse counts to zero
    rightPulseCount = 0;
    __enable_interrupt();
    // if there were any pulses, calculate the new position
    if (leftCount || rightCount)
    {
        DTheta = (rightCount - leftCount)* 0.06686; // do the position calculation
        if(leftCount == rightCount)
        {
          dx = 0.007114 * rightCount * cos(ppos->h);
          dy = 0.007114 * rightCount * sin(ppos->h);
        
        }
        else if(leftCount != rightCount)
        {
          dx = 0.0532*((rightCount + leftCount)/(rightCount - leftCount))* (sin (ppos->h +DTheta) - sin(ppos->h));
                                                                         
          dy = 0.0532*((rightCount + leftCount)/(rightCount - leftCount))* (cos (ppos->h) - cos(ppos->h +DTheta));
        
        }
        
         ppos->h = ppos->h + DTheta;
         ppos->x = ppos->x + dx;
         ppos->y = ppos->y + dy;
        // display the new position (convert heading to degrees)
        sprintf(str, "POS,%7.3f,%7.3f,%7.1f,%3d,%3d\r\n", ppos->x, ppos->y,
            ppos->h * 180. / PI, leftCount, rightCount);
        OutputString(str);
    }
}

#if USE_SERIAL_INTERRUPTS == 1
// transmit serial string USING INTERRUPTS
void OutputString(char* str)
{
    int length = strlen(str);
    UCSR0B_UDRIE0 = 0; // disable serial port 0 UDRE interrupt
    // check for too many chars
    if (count + length >= BUF_MAX)
    {
        UCSR0B_UDRIE0 = 1; // enable serial port 0 UDRE interrupt
        return;
    }
    // write the characters into the buffer
    for (int n = 0; n < length; n++)
    {
        buffer[tail] = str[n];
        tail++;
        if (tail >= BUF_MAX)
        {
            tail = 0;
        }
    }
    count += length;
    UCSR0B_UDRIE0 = 1; // enable serial port 0 UDRE interrupt
}
#else
// transmit serial string NOT USING INTERRUPTS
void OutputString(char* str)
{
    int length = strlen(str);
    // for each character in the string
    for (int n = 0; n < length; n++)
    {
        // wait while the serial port is busy
        while (!UCSR0A_UDRE0);
        // transmit the character
        UDR0 = str[n];
    }
}
#endif

/*******************************************************************************
* INTERRUPT FUNCTIONS
*******************************************************************************/
#pragma vector = INT0_vect
__interrupt void LeftCounterISR(void) // left wheel pulse counter
{
    leftPulseCount++;
}

#pragma vector = INT1_vect
__interrupt void RightCounterISR(void) // right wheel pulse counter
{
    rightPulseCount++;
}

#if USE_SERIAL_INTERRUPTS == 1
#pragma vector = USART0_UDRE_vect
__interrupt void Serial0UDREmptyISR(void) // serial DRE (transmit) interrupt
{
    if (count > 0) // if there are more characters
    {
        UDR0 = buffer[head]; // transmit the next character
        // adjust the buffer variables
        head++;
        if (head > BUF_MAX)
        {
            head = 0;
        }
        count--;
    }
    if (count == 0) // if there are no more characters
    {
        UCSR0B_UDRIE0 = 0; // then disable serial port 0 UDRE interrupt
    }
}
#endif
