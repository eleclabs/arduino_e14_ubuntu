/* Pins used for control signals */
#define STEP 14
#define DIRECTION 27
//#define ENABLE 27



#define FORWARD HIGH
#define REVERSE LOW

/* Change this values to alter the clock speed */
#define SPEED 1

void setup() 
{
  //pinMode(ENABLE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(STEP, OUTPUT);

  /* Pull the enable pin low to enable the driver */
  //digitalWrite(ENABLE, LOW); //LOW
}


void loop() 
{
  /* The the rotational direction to the forward direction */
  digitalWrite(DIRECTION, FORWARD);

  /* Keep stepping the motor in an infinite loop */
  while(1)
  {
    digitalWrite(STEP, HIGH);   
    delay(SPEED);              
    digitalWrite(STEP, LOW);    
    delay(SPEED);        
  }
}