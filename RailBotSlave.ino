/*****************************************************************************

    File Overview:
    
    Authors:
    Delivery Date:
    
    File Name:  RailBot.ino
    
 *****************************************************************************
 $LOG$
 *****************************************************************************
 $NOTES$
 1) Backwards and forwards are relative to user specified intial start
 *****************************************************************************
 $REFERENCES$
 *****************************************************************************/

/*-----( ARDUINO MEGA2650 PIN CONFIGURATION )-----*/

/* LASER - UNI-T

*/


/* Radio Modules - nRF24L01
  Modules See:
  http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
  
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 9
   4 - CSN to Arduino pin 10
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED
   
 - V1.00 11/26/13
   Based on examples at http://www.bajdi.com/
   Questions: terry@yourduino.com
*/


/*-----( Needed libraries )-----*/
#include <SPI.h>           // SPI bus Library
#include "RF24.h"          // RF Module Library
#include "printf.h"        // RF Printf Library
#include "nRF24L01.h"      // RF Module Definitions

/*-----( Pin Definitions )-----*/
// Digital Components
#define RF_CS_P                    9 // RF Chip Select (out) => pin9
#define RF_CSN_P                   8 // RF_ (out) => pin8

// Analong Components
#define MOTOR_A_P                  7 // PWM A (out) => pin7
#define MOTOR_B_P                  6 // PWM B (out) => pin6
#define MOTOR_EN_P                 5 // Motor enable => pin5

// Other Deinitions
#define MAX_SPEED                 220 // Motor max PWM (count)
#define MOTOR_SPEED               220 // Normal motor speed (pwm)
#define MOTOR_ADJ_SPEED           200 // Ajusting motor speed (pwm)
#define ENCODER_DIA_INCHES (PI*1.625) // Diameter of encoder in inches (inches)

#define MOVE 0
#define RETURN 1
#define DONE 2


/*-----( Global Variables )-----*/

// Motor Control
long count;                           // Current number of interrupts from encoder
long currentCount;                    // Position at beginning of call to move()
long countIncrement;                  // Number of counts per user interval spec
long currPosition = 1;                // Position of the robot currently (measurement number)
long totalCount;
long loopCount;                       // Number of times count increment met
int  motorDirection;                   // Direction the robot is moving
volatile boolean isMoving;            // Whether or not robot is currently moving. Declared as volatile so

// Debug/Testing
int testCounter;                                      
int feet;
int inches;

/*-----( Instantiate Radio )-----*/
RF24 radio(RF_CS_P,RF_CSN_P); // Create a Radio
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
long transmitSuccess = 1;


/*-----( ADRDUINO FUNCTIONS )-----*/
/*
  Setup: Set up all required I/O and global variables
*/
void setup()
{    
  /* Laser initialization */
  Serial.begin(115200);   // runs with 115200 baud

  // Motor pin modes
  pinMode( MOTOR_A_P, OUTPUT );
  pinMode( MOTOR_B_P, OUTPUT );
  pinMode( MOTOR_EN_P, OUTPUT );

  // Motor Initialize
  digitalWrite( MOTOR_A_P, HIGH );
  digitalWrite( MOTOR_B_P, LOW );
  analogWrite( MOTOR_EN_P, 0 );

  // Rotary encoder interrupt
  attachInterrupt(0, countInt, CHANGE);

  // RF Initialization
  pinMode( RF_CS_P, OUTPUT );
  pinMode( RF_CSN_P, OUTPUT );
  printf_begin();
  radio.begin();
  Serial.println("RF Module information:");
  radio.printDetails();
  
  radio.setRetries(15,15);
  
  radio.openReadingPipe(1,pipes[1]);
  radio.openWritingPipe(pipes[0]);
  
  radio.startListening();
  
  Serial.println("Waiting for parameters...");
  
  getParameters();
  
  Serial.print("total count: ");
  Serial.println(totalCount);
  
  Serial.print("count increment: ");
  Serial.println(countIncrement);

  loopCount = 0;
  
  // Motor control globals
  count = 0;
  loopCount = 0;
  totalCount = 0;
  motorDirection = 1;
  
}//--( end setup )---

/*
  loop: MAIN ENTRY FOR MEGA, RUNS CONSTANTLY
*/
void loop()
{ 
  
  int action;
  
  action = wait();
  
  if (action == MOVE) {
     driveMotor();
     resetMotor();
     signal();
  } else if (action == RETURN) {
     returnToStart();
     signal();
  }
  delay(2000);
}//--( end main loop )---


/*-----( USER FUNCTIONS )-----*/

int wait() {
  
  int command;
  
  while (!radio.available());
  
  while(!radio.read(&command, sizeof(int)));
  
  return command;
  
}
  
void signal() {
  
  int command = DONE;
  
  radio.stopListening();
  
  while(!radio.write(&command, sizeof(int)));
  
  radio.startListening();
  
  return;
}
  
  void getParameters(){
  
  long code;
  long value;
  
  boolean flag1 = false;
  boolean flag2 = false;
  
  radio.startListening();

  while(true) {
  
    while(!radio.available());
  
    radio.read(&code, sizeof(long));
    Serial.println(code);
  
    while(!radio.available());
  
    radio.read(&value, sizeof(long));
    Serial.println(value);
    
    if (code == 0) {
        totalCount = value;
        flag1 = true;
    } else if (code == 1) {
         countIncrement = value;
         flag2 = true;
    }
    
    if (flag1 && flag2) {
       break;
    } 
 
  }
   return;
    
  }  
  
  
  
  
  
/*
  Drive Motor: An algorithm to efficiently get the robot to the next position. This
    function moves the robot the distance represented by the value in countIncrement.
    This variable will be set during the initialization based on the increment distance
    configured by the user before starting the survey. The idea of this function is to
    set the speed to MOTOR_SPEED. The robot will move until the countIncrement is met.
    At this point, it is checked if the robot has made it to the correct distance. If
    There is any overshoot or undershoot, the robot will auto correct until it reaches
    the correct location.
*/
void driveMotor ()
{ 
  // Declare/Initialize adjustment parameters
  int temp_speed = MOTOR_SPEED;
  int temp_direction = motorDirection;
  
  // Save the current position
  currentCount = count;
  int targetLocation = currentCount + countIncrement;
  
  // debug
  Serial.print("driveMotor - Saving count: Current count = ");
  Serial.println(currentCount);
  
  // Drive the next location
  do
  {
    // debug
    Serial.print("driveMotor - Moving with speed: ");
    Serial.println(temp_speed);
      
    // Set normal speed
    analogWrite( MOTOR_EN_P, temp_speed );
    
    // Robot is now moving
    isMoving = true;
        
    // Begin counting traversal
    while( isMoving ); // Wait until ISR stops robot
    delay(1000);     // Slowdown time
      
    Serial.println("driveMotor - Done Moving.");
      
    // After robot has stopped, check if not at correct location
    if( count != targetLocation )
    {
      //debug
      Serial.println("driveMotor - Not at the target location");
        
      // Set adjustment speed (slower then normal speed)
      temp_speed = MOTOR_ADJ_SPEED;
        
      // Check if overshoot, then must change direction 
      if(count > targetLocation & temp_direction == motorDirection)
      {
        Serial.println("driveMotor - ...overshot it");
        changeDirection();
      }
      // Overshot while trying to adjust
      else if (count < targetLocation & temp_direction == -motorDirection)
      {
        Serial.println("driveMotor - ...overshot it");
        changeDirection();  
      }
    }
    // Found the correct location
    else
    {
     Serial.println("Found the location!!");
     // Verify direction is still forward
     if(motorDirection != temp_direction)
     {
       Serial.println("Changing direction after adjusting in opposite direction");
       changeDirection();
     }
     break; 
    }
      
  } while(true);
  
  return;
}



/*
  Change Direction: Reconfigures the motor to drive the opposite direction
*/
void changeDirection()
{
  Serial.println("changeDirection - Changing Direction...");
  analogWrite( MOTOR_EN_P, 0 );
  
  // Already moving forward, so..
  if ( motorDirection == 1 )
  {
    // Set to move backwards
    motorDirection = -1;
    digitalWrite( MOTOR_A_P, LOW );
    digitalWrite( MOTOR_B_P, HIGH );
    Serial.print("changeDirection - Motor direction switched = ");
    Serial.println(motorDirection);
  }
  // Already moving backward, so...
  else if ( motorDirection == -1 )
  {
    // Set to move forwards
    motorDirection = 1;
    digitalWrite( MOTOR_A_P, HIGH );
    digitalWrite( MOTOR_B_P, LOW );
    Serial.print("changeDirection - Motor direction switched = ");
    Serial.println(motorDirection);
  }
}

// Turns off motor and restores it to its original direction before the ISR braked it
void resetMotor() {
  
  delay(100);
  
  // Turn off motor
  analogWrite( MOTOR_EN_P, 0 );
  
  if ( motorDirection == 1 )
  {
    digitalWrite( MOTOR_A_P, HIGH );
    digitalWrite( MOTOR_B_P, LOW );
  }
  else if ( motorDirection == -1 )
  {
    digitalWrite( MOTOR_A_P, LOW );
    digitalWrite( MOTOR_B_P, HIGH );
  }
  
}

void returnToStart() {
  
  if (motorDirection == 1) {
    changeDirection();
  }
  
  currentCount = count;
  countIncrement = -(totalCount - 10);

  analogWrite( MOTOR_EN_P, MOTOR_SPEED );
  
  isMoving = true;
  
  while(isMoving);
  
  return;
  
}

//ISR for the rotary encoder
//Increments the counter variable by 1 every time it is triggered on both the rising
//and falling edge of the signal. This translates into 16 interrupts per rotation of the 
//disc. With a 1 5/8 diameter wheel, this equals 0.319068" per interrupt (1 5/8 * pi / 16)
//Brakes motor once desired distance was traveled
void countInt(){

  // Moving forward, increment counter
  if ( motorDirection == 1 )
  {
    count++;
  }
  // Moving backward, becrement counter
  else if ( motorDirection == -1 )
  {
    count--;
  }
  
  //Serial.print("CountInt - Count changed: Count = ");
  //Serial.println(count);  
  
  // Service routine to stop the robot
  if ( count == (currentCount + countIncrement) )
  { 
    Serial.println("ISR Stopping motor");
    // Toggle one motor pin to make both either 0 or 1
    digitalWrite(MOTOR_A_P, !digitalRead( MOTOR_A_P ) );
    
    // Brake motor
    analogWrite(MOTOR_EN_P, 255);
    
    // Robot is no longer moving
    isMoving = false;
   }
}


