#include <PID_v1.h>

double kP = 150;
double kI = 300;
double kD = 10;

double setpoint, input, output;   // PID variables
PID pid(&input, &output, &setpoint, kP, kI, kD, DIRECT); // PID setup




unsigned int counter=0;

int Motor = 6;  // Motor Driver Pin
int motorspeed;
int desired_speed = 0;
int rotation;
int RPM;


int value = 49910;   //Preload timer value (49910 for 1 seconds)


int potvalue;

void docount()  // counts from the speed sensor
{
  counter++;  // increase +1 the counter value
} 



void setup() 
{


  Serial.begin(9600);
  
  pinMode(Motor, OUTPUT); 

/*    Timer1 (16-bit timer Setup) */

  noInterrupts();                       // disable all interrupts


  TCCR1A = 0;

  TCCR1B = 0;

  TCNT1 = value;                        // preload timer

  TCCR1B |= (1 << CS10)|(1 << CS12);    // 1024 prescaler 

  TIMSK1 |= (1 << TOIE1);               // enable timer overflow interrupt ISR

  interrupts();                         // enable all interrupts


  attachInterrupt(0, docount, RISING);  // increase counter when speed sensor pin goes High


  // PID Setup
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0,255);
    pid.SetSampleTime(10);


} 

ISR(TIMER1_OVF_vect)                    // interrupt service routine for overflow

{
  
  Serial.print("Motor Speed: "); 
  rotation = (counter / 20);  // divide by number of holes in Disc
  Serial.print(rotation,DEC);  
  Serial.println(" RPS"); 
  RPM = rotation*60;
  Serial.print(RPM,DEC);  
  Serial.println(" RPM"); 
  Serial.print("Desired Speed: ");
  Serial.print(desired_speed,DEC);
  TCNT1 = value;                                // preload timer
  counter=0;  //  reset counter to zero

}


void loop()
{
  potvalue = analogRead(A0);  // Potentiometer connected to Pin A0
  desired_speed = map(potvalue, 0, 1023, 0, 50);

  // PID vars
  setpoint =  desired_speed ; 
  input = rotation;
  pid.Compute();
  motorspeed = output;
  analogWrite(Motor, motorspeed);  // set speed of motor (0-255)
}