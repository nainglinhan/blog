//Left Motor 
#define leftMotor_1A 5   // Left Motor Pin 1A connected to Pin 5
#define leftMotor_1B 6   // Left Motor Pin 1B connected to Pin 6
//Function Connection
#define leftMotor_PWM leftMotor_1A    // Left Motor PWM Speed Control
#define leftMotor_DIR leftMotor_1B    // Left Motor Direction Control

//Right Motor
#define rightMotor_1A 10 // Right Motor Pin 1A
#define rightMotor_1B 11 // Right Motor Pin 1B
//Function Connection 
#define rightMotor_PWM rightMotor_1A    // Right Motor PWM Speed Control
#define rightMotor_DIR rightMotor_1B    // Right Motor Direction Control

//Speed of Motor
#define PWM_SLOW_L 80
#define PWM_NORMAL_L 130

#define PWM_SLOW_R  50   // Slow Speed PWM Duty Cycle
#define PWM_NORMAL_R 100 //Normal Speed PWM Duty Cycle

//Ping Sensor
#define echo 8          // Echo Pin of Ping
#define trig 9          // Trigger Pin of Ping 
long duration, distance;
int obj = 20;           //Obstacle at 25cm

//Line Tracking Sensors
const int lineSensor1 = 12;
const int lineSensor2 = 7;
const int lineSensor3 = 4;
const int lineSensor4 = 3;
const int lineSensor5 = 2;

void setup()
{
  Serial.begin(9600);

  //Motors
  pinMode(leftMotor_PWM, OUTPUT);
  pinMode(leftMotor_DIR, OUTPUT);
  pinMode(rightMotor_PWM, OUTPUT);
  pinMode(rightMotor_DIR, OUTPUT);
  
  digitalWrite(leftMotor_PWM, LOW);
  digitalWrite(leftMotor_DIR, LOW);
  digitalWrite(rightMotor_PWM, LOW);
  digitalWrite(rightMotor_DIR, LOW);

  //Ping Sensor
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);

  //Line Sensors
  pinMode(lineSensor1, INPUT);
  pinMode(lineSensor2, INPUT);
  pinMode(lineSensor3, INPUT);
  pinMode(lineSensor4, INPUT);
  pinMode(lineSensor5, INPUT);  
}

void loop ()
{
  distance = ping();

  if (distance < obj )    // Object is Detected
  {
    Serial.println("Objecte is Detecteed!");
    M_Stop ();            // Stop The Motor
  }

  else 
  {
    Serial.println("Clear");
    M_Control();          // Control The Motor
  }
  delayMicroseconds (1000);
}



long ping() //This is for Ultrasonic sensor
{
  digitalWrite(trig,LOW);        // first ...no sound wave is sent out for 2 microseconds
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);        // Then sent out the sound wave for 5 microseconds
  delayMicroseconds(5);
  digitalWrite(trig,LOW);
  duration= pulseIn(echo,HIGH);
  return duration / 29 / 2;        // changing the microsecond to centimeter
}

void M_Control ()
{
  int ls1 = digitalRead (lineSensor1);
  int ls2 = digitalRead (lineSensor2);
  int ls3 = digitalRead (lineSensor3);
  int ls4 = digitalRead (lineSensor4);
  int ls5 = digitalRead (lineSensor5);

  Serial.println(ls1);
  Serial.println("MOTOR CONTROL");

  if (ls1==0 && ls2==0 && ls3 ==1 && ls4==0 && ls5==0)
  {
    M_Forward();
  }
  
  else if (ls1==0 && ls2==1 && ls3 ==0 && ls4==0 && ls5==0)
  {
    M_Left_1();
  }
 
  else if (ls1==1 && ls2==0 && ls3 ==0 && ls4==0 && ls5==0)
  {
    M_Left_2();
  }

  else if (ls1==0 && ls2==0 && ls3==0 && ls4==1 && ls5==0)
  {
    M_Right_1();
  }

  else if (ls1==0 && ls2==0 && ls3==0 && ls4==0 && ls5==1)
  {
    Serial.println("RIGHT");
    M_Right_2();
  }

  //ERRORS
  
  else if (ls1==0 && ls2==1 && ls3 ==1 && ls4==0 && ls5==0)
  {
    M_Left_1();
  }
 
  else if (ls1==1 && ls2==1 && ls3 ==0 && ls4==0 && ls5==0)
  {
    M_Left_2();
  }

  else if (ls1==1 && ls2==1 && ls3 ==1 && ls4==0 && ls5==0)
  {
    M_Left_2();
  }

  else if (ls1==0 && ls2==0 && ls3==1 && ls4==1 && ls5==0)
  {
    M_Right_1();
  }

  else if (ls1==0 && ls2==0 && ls3==0 && ls4==1 && ls5==1)
  {
    Serial.println("RIGHT");
    M_Right_2();
  }

 else if (ls1==0 && ls2==0 && ls3==1 && ls4==1 && ls5==1)
  {
    Serial.println("RIGHT");
    M_Right_2();
  }
  
  else 
  {
    M_Stop();
  }
}

void M_Forward ()
{
   Serial.println("Robot Is Moving Forward!");    
  // Left Motor 
  digitalWrite(leftMotor_DIR, HIGH);             // Left Motor Direction = Forward 
  analogWrite(leftMotor_PWM, 255-PWM_NORMAL_L);  // Left Motor Speed = NORMAL
  // Right Motor 
  digitalWrite(rightMotor_DIR, HIGH);             // Right Motor Direction = Forward 
  analogWrite(rightMotor_PWM, 255-PWM_NORMAL_R);  // Right Motor Speed = NORMAL
}

void M_Left_1 ()
{
   Serial.println("Robot Is Turn Left");
  // Left Motor 
  digitalWrite(leftMotor_DIR, HIGH);             // Left Motor Direction = Forward 
  analogWrite(leftMotor_PWM, 255-PWM_SLOW_L);     // Left Motor Speed = Slow
  // Right Motor 
  digitalWrite(rightMotor_DIR, HIGH);             // Right Motor Direction = Forward 
  analogWrite(rightMotor_PWM, 255-PWM_NORMAL_R);  // Right Motor Speed = NORMAL
  }

void M_Left_2 ()
{
  Serial.println("Robot Is Turn Left Sharply");
  // Left Motor STOP
  digitalWrite(leftMotor_DIR, LOW);             
  analogWrite(leftMotor_PWM, LOW);  
  // Right Motor 
  digitalWrite(rightMotor_DIR, HIGH);             // Right Motor Direction = Forward 
  analogWrite(rightMotor_PWM, 255-PWM_NORMAL_R);  // Right Motor Speed = NORMAL
}

void M_Right_1 ()
{
  Serial.println("Robot Is Turn Right");
  // Left Motor 
  digitalWrite(leftMotor_DIR, HIGH);             // Left Motor Direction = Forward 
  analogWrite(leftMotor_PWM, 255-PWM_NORMAL_L);  // Left Motor Speed = NORMAL
  // Right Motor 
  digitalWrite(rightMotor_DIR, HIGH);             // Right Motor Direction = Forward 
  analogWrite(rightMotor_PWM, 255-PWM_SLOW_R);  // Right Motor Speed = SLOW
}

void M_Right_2 ()
{
  Serial.println("Robot Is Turn Right Sharply!"); 
  // Left Motor 
  digitalWrite(leftMotor_DIR, HIGH);             // Left Motor Direction = Forward 
  analogWrite(leftMotor_PWM, 255-PWM_NORMAL_L);  // Left Motor Speed = NORMAL
  // Right Motor STOP
  digitalWrite(rightMotor_DIR, LOW);          
  analogWrite(rightMotor_PWM, LOW);  
}

void M_Stop ()
{
 //STOP BOTH MOTORS
  digitalWrite(leftMotor_DIR, LOW);         
  digitalWrite(leftMotor_PWM, LOW);
  digitalWrite(rightMotor_DIR, LOW);              
  digitalWrite(rightMotor_PWM, LOW);
}

