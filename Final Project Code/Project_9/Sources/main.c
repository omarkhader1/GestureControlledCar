#include <hidef.h>      /* common defines and macros */
#include "derivative.h" /* derivative-specific definitions */

#define FORWARD1  '3.0'
#define FORWARD2  '4.0'
#define FORWARD3  '5.0'
#define REVERSE  '6.0'
#define LEFT     '2.0'
#define Right    '1.0'
#define CRUISE   '7'
#define STOP     '0.0'

#define TRIGGER_PIN  PTM_PTM2   // Ultrasonic trigger pin
#define ECHO_PIN     PTM_PTM3   // Ultrasonic echo pin
#define SERVO_PIN    PTM_PTM4   // Servo control pin

#define TRIGGER_DELAY 10   // Trigger pulse width in microseconds
#define SOUND_SPEED 343   // Speed of sound in m/s
#define DIST_THRESHOLD 5 // Distance threshold in cm

// Function prototypes
void interrupt 20 receive(void);
void initUART(void);
void initPWM(void);
void initTimer(void);  
void initUltrasonic(void);
void RTI_init(void);
void setPWMDutyCycle(char channel, int dutyCycle);
void setServoPosition(int angle);
void moveForward(int speed);
void moveReverse(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stop(void);
void cruiseControl(void);
void interrupt 7 calculateSpeed(void);
void updatePID(void);
unsigned int measureDistance(void);
void checkObstacle(void);
void myDelay2 (unsigned int val);

void interrupt 9 hallSensorISR1(void);
void interrupt 10 hallSensorISR2(void);
void motorinit(void);
void PLLinit(void);
void myDelay(void);

// Global variables for PID co0100;

volatile int currentSpeed1 = 0;
volatile int currentSpeed2 = 0;
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
volatile char receivedCommand = STOP;
int topspeed=0;
int targetSpeed=0;
int Kp = 1, Ki = 0, Kd = 0;
int error, previousError = 0, integral = 0, derivative;
int pidOutput;
int overflow;
int flag;
unsigned int j;
void interrupt 16 handler2(){     //interrupt service routine for the TCNT overflow interrupt
     if(flag==1){
       TFLG2=0x80; // clear the TOF flag
       overflow++; 
     }

}
void main(void) 
{
  PLLinit();
  motorinit();
  initUART();
  initPWM();
  RTI_init();
  initUltrasonic();  
  EnableInterrupts;
  
  overflow=0;
  flag=0; 
  TSCR1=0x90; // enable the TCNT counter and Fast Flag clear
  TSCR2=0x05; // disable the overflow interrupt and set the prescale to 32
  TIOS=TIOS & 0xF9;
  TCTL4=0x28; // latch the counter on the rising edge of IC1
  TFLG1=0x06; // clear the IC1 flag
  TIE=0x06; // enable IC1 & IC2 Interrupt
  for(;;) {

  }
}

void RTI_init(void){
RTICTL=0x7F;
CRGINT = CRGINT | 0x80;// enabled the RTI interrupt
}
void PLLinit(void){
    SYNR=0x02;
    REFDV=0x01;
    PLLCTL=0x60;
    while(!(CRGFLG & 0x08));
    CLKSEL=0x80;
}

void motorinit(void){
  DDRB=0xFF;
  PORTB=0x0A;
  PTP=0x03; 
  myDelay2(500);
  topspeed = ((pulseCount1 + pulseCount2)/2)*120;
  PTP = 0x00;
  PORTB = 0x00; 
}
void initUART(void) {

      SCI0CR1=0x00;
      SCI0CR2=0x24;
      SCI0BDL=156;//9600 baud
      SCI0BDH=0x00;       // Return received byte
}

void initPWM(void) {
  PWMCLK=0x03; // select SA
  PWMPOL=0x0F; // Start high (1 polirity)// we are using a nMOS
  PWMPRCLK=0x47; //prescale the E-clock by 128 --> 24M/128 = 187.5 Khz
  PWMSCLA=0x04;//  prescale A by 8 (4*2) to get SA --> 187.5KHz/8 = 23.43KHz
  PWMCTL=0x2C; // select 8-bit mode and disable PWM in wait and freeze modes
//i.e each count will take this amount of time: (1/24MHz)*128*8 = 42.666 microseconds
  
  PWMPER0=235; // this will get us 10.02 ms period (10.02 ms =235 counts * 42.666 micro Second per count)
  PWMDTY0=235/2; // 50% duty cycle (initially)
  PWMPER1=235; // this will get us 10.02 ms period (10.02 ms =235 counts * 42.666 micro Second per count)
  PWMDTY1=235/2; // 50% duty cycle (initially)
  PWMPER2=30000>>8; //highest 8 bit of the Period
  PWMPER3=30000 & 0x00FF; // lowest 8 bit of the period
  
    
  // start with the servo pointing 0 degrees
  PWMDTY2=2000>>8; // highest 8 bit of the duty
  PWMDTY3= 2000 & 0x00FF; //lowest 8 bit of the duty
  PWMCNT3=0x00;
  PWMCNT2=0x00;
  PWME=0x0F;
}

void initTimer(void) {
  TSCR1 = 0x80;      // Enable timer and fast flag clear
  TSCR2 = 0x06;      // Enable timer interrupt, prescale by 64
  TIOS |= 0x01;      // Output compare on channel 0
  TC0 = 37500;       // Set up timer compare value for 100ms (assuming 24MHz clock)
  TIE |= 0x01;       // Enable interrupt for channel 0
}



void initUltrasonic(void) {
  DDRM |= 0x04;    // Set PTM2 (trigger) as output
  DDRM &= ~0x08;   // Set PTM3 (echo) as input
}



void setPWMDutyCycle(char channel, int dutyCycle) {
  int scaledDutyCycle = (dutyCycle * 255) / topspeed; // Scale top speed to 0-255
  switch(channel) {
    case 0: PWMDTY0 = scaledDutyCycle; break;
    case 1: PWMDTY1 = scaledDutyCycle; break;
   // case 2: PWMDTY2 = scaledDutyCycle; break;
   // case 3: PWMDTY3 = scaledDutyCycle; break;
  
    default: break;
  }
}

void setServoPosition(int angle) {
  int pulseWidth = 1000 + (angle * 1000 / 180); // Pulse width in microseconds (1000us to 2000us)
  int dutyCycle = (pulseWidth * 100) / 20000; // Convert to duty cycle percentage (0-100%)
  PWMDTY2=dutyCycle>>8; // highest 8 bit of the duty
  PWMDTY3= dutyCycle & 0x00FF; //lowest 8 bit of the
}

void moveForward(int speed) {
  // Set PWM duty cycle to control motor speed
  PORTB =0x0A;
  setPWMDutyCycle(0, speed); // Assume channel 0 controls forward movement
  setPWMDutyCycle(1, speed);     // Set reverse channel to 0
}

void moveReverse(int speed) {
  // Set PWM duty cycle to control motor speed
  PORTB =0x05; 
  setPWMDutyCycle(0, speed);     // Set forward channel to 0
  setPWMDutyCycle(1, speed); // Assume channel 1 controls reverse movement
}

void turnLeft(int speed) {
  // Implement motor control logic to turn left
  PORTB =0x0A;
  setPWMDutyCycle(0, speed); // Example: turn left at 50% speed
  setPWMDutyCycle(1, speed/2);
}

void turnRight(int speed) {
  // Implement motor control logic to turn right
  PORTB =0x0A;
  setPWMDutyCycle(0, speed/2); // Example: turn left at 50% speed
  setPWMDutyCycle(1, speed);
}

void stop(void) {
  // Implement motor control logic to stop
  
  setPWMDutyCycle(0, 0);
  setPWMDutyCycle(1, 0);
  
  
}

void cruiseControl(void) {
  // Implement logic to set the cruise control speed
  // For simplicity, toggle between a set speed and stop
  if (targetSpeed == 0) {
    targetSpeed = topspeed/2; // Set to desired cruise speed
  } else {
    targetSpeed = 0;  // Reset to 0 to stop
  }
}

void interrupt 7 calculateSpeed(void) {    // RTI that triggers each 22ms 
  // Calculate the wheel speeds based on pulse counts
  // Reset pulse counts after calculation
  currentSpeed1 = pulseCount1*2728; // You can scale this as needed
  currentSpeed2 = pulseCount2*2728;
  pulseCount1 = 0;
  pulseCount2 = 0;
  j++;
}

void updatePID(void) {
  // Calculate the PID control value
  error = targetSpeed - ((currentSpeed1 + currentSpeed2) / 2);
  integral += error /10; // Integral term (0.1 represents the time interval in seconds)
  derivative = (error - previousError) *10; // Derivative term
  pidOutput = (int)(Kp * error + Ki * integral + Kd * derivative);
  previousError = error;
  
  // Ensure PID output is within valid range (0-100%)
  if (pidOutput > 100) pidOutput = 100;
  if (pidOutput < 0) pidOutput = 0;
  
  moveForward(pidOutput); // Adjust motor speed based on PID output
}

unsigned int measureDistance(void) {
  unsigned int duration, distance;
  // Send trigger pulse
  TRIGGER_PIN = 1;
  myDelay2(TRIGGER_DELAY);
  TRIGGER_PIN = 0;
  
  // Wait for echo start
  while (!ECHO_PIN);
  // Measure echo duration
  TFLG1 = 0x01; // Clear timer overflow flag
  TCNT = 0;
  while (ECHO_PIN && !TFLG1) duration = TCNT;
  
  // Calculate distance in cm
  distance = (duration * SOUND_SPEED) / (2 * 10000); // Convert to cm
  return distance;
}

void checkObstacle(void) {
  unsigned int distance;
  int angle;
  for (angle = 0; angle <= 180; angle += 10) {
    setServoPosition(angle);
   
    myDelay2(100);
    
    distance = measureDistance();
    if (distance < DIST_THRESHOLD) {
      stop();
      break;
    }
  }
}

void interrupt 20 receive(void){
      receivedCommand = SCI0DRL;// Read the command from UART
  
  // Process the received command
  switch(receivedCommand) {
    case FORWARD1:
      targetSpeed = topspeed/3; 
      break;
    case FORWARD2:
      targetSpeed = (topspeed*2)/3; 
      break;
    case FORWARD3:
      targetSpeed = topspeed; 
      break;  
    case REVERSE:
      moveReverse(topspeed/3); // 
      break;
    case LEFT:
      turnLeft(topspeed/3);
      break;
    case Right:
      turnRight(topspeed/3);
      break;
    case CRUISE:
      cruiseControl();
      break;
    case STOP:
      stop();
      targetSpeed = 0;
      break;
    default:
      stop();
      targetSpeed = 0;
      break;
  }
  
  
  updatePID();
  checkObstacle(); // Check for obstacles
}


void interrupt 9 hallSensorISR1(void) {
  pulseCount1++; // Increment pulse count for first wheel
  TFLG1 = 0x02;  //clear flag
}

void interrupt 10 hallSensorISR2(void) {
  pulseCount2++; // Increment pulse count for second wheel
  TFLG1 = 0x04;  //clear flag
}

void myDelay(void){
    unsigned char i;
    unsigned int j;
    
    for(i=0;i<20;i++){
          for(j=0;j<20000;j++){
          
              i=i;
              j=j;
          }
    
    
    
    }

}

void myDelay2 (unsigned int val){
 j=0;
 while(j*22<val);
}