/* Pins used for control signals */
#define STEP1 14
#define DIRECTION1 27
#define STEP2 13
#define DIRECTION2 12
//#define ENABLE 27



#define FORWARD HIGH
#define REVERSE LOW

/* Change this values to alter the clock speed */
#define SPEED 1

void setup() 
{
  //pinMode(ENABLE, OUTPUT);
  pinMode(DIRECTION1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(DIRECTION2, OUTPUT);
  pinMode(STEP2, OUTPUT);

  /* Pull the enable pin low to enable the driver */
  //digitalWrite(ENABLE, LOW); //LOW
}


void loop() 
{
  /* The the rotational direction to the forward direction */
  digitalWrite(DIRECTION1, FORWARD);
  digitalWrite(DIRECTION2, FORWARD);

  /* Keep stepping the motor in an infinite loop */
  while(1)
  {
    digitalWrite(STEP1, HIGH);   
    digitalWrite(STEP2, LOW); 
    delay(SPEED);              
    digitalWrite(STEP1, LOW);    
    digitalWrite(STEP2, HIGH); 
    delay(SPEED);        
  }
}