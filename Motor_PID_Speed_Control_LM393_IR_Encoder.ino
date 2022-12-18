#include <PID_v1.h>

double kP = 0.009;
double kI = 0.02;
double kD = 1.4;

double setpoint, input, output;   // PID variables
PID pid(&input, &output, &setpoint, kP, kI, kD, DIRECT); // PID setup




double counter=0;

int Motor = 6;  // Motor Driver Pin
double motorspeed;
double desired_speed = 0;
double rotation;
double RPM;


float value = 49911;   //Preload timer value (49910 for 1 seconds)


int potvalue;

void docount()  // counts from the speed sensor
{
  counter++;  // increase +1 the counter value
} 



void setup() 
{


  Serial.begin(115200);
  
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
    pid.SetMode(1);
    pid.SetOutputLimits(0,255);
    pid.SetSampleTime(1);


} 

ISR(TIMER1_OVF_vect)                    // interrupt service routine for overflow

{
   
  rotation = (counter / (20.0));
  RPM = rotation*60;
  Serial.print("Setpoint:");  
  Serial.print(desired_speed/10);
  Serial.print(','); 
  Serial.print("Measured Speed:");  
  Serial.print(RPM/10);
  Serial.print(',');
  Serial.println();

  
  TCNT1 = value;                                // preload timer
  counter=0;  //  reset counter to zero

}


void loop()
{
  potvalue = analogRead(A0);  // Potentiometer connected to Pin A0
  desired_speed = map(potvalue,0,1023,0,2000);

  // PID vars
  setpoint =  desired_speed ; 
  input = RPM;
  pid.Compute();
  motorspeed = output;
  analogWrite(Motor, motorspeed);  // set speed of motor (0-255)
}