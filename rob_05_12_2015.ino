/*
 * A0 - front
 * A1 - left
 * A2 - right
 * 
 * E2(D10), M2(D11) - (servo) - left
 * E1(D9), M1(D8) - (servo) - right,    
 */

#define E1 9
#define M1 8
#define E2 10
#define M2 11
#define MAXSPEED 60
// Right wheel adjustment
#define ERR 15

// Sensor read transform
#define VOLTS_PER_UNIT    .0049F        // (.0049 for 10 bit A-D) 
float volts;
float cm;  

int AFront = 0, ALeft = 1, ARight = 2; 

int DISTANCE = 400; 
int TRESHOLD = 10; 
int DIFF = 200;

void setup()
{
//  attachInterrupt(1, callbackFuncBal, CHANGE);
//  attachInterrupt(0, callbackFuncJobb, CHANGE);
  Serial.begin(9600);
}

float convertRead(int value)
{
  volts = (float)value * VOLTS_PER_UNIT; // ("proxSens" is from analog read) 
  cm = 60.495 * pow(volts,-1.1904);     // same in cm
  if (volts < .2) 
  {
    cm = -1.0;       // out of range
  }
  return cm;
}

void accRobot()
{
    for(int i=0; i < MAXSPEED; i+=1)
    {
      digitalWrite(M1,LOW);
      digitalWrite(M2,LOW);
      analogWrite(E1,i);
      analogWrite(E2,i+15);
      delay(10);
    }
}

void slowRobot()
{
  int i = MAXSPEED;//analogRead(E2);
  while (i > 0)
  {
      digitalWrite(M1,LOW);
      digitalWrite(M2,LOW);
      analogWrite(E1,i);
      analogWrite(E2,i);
      delay(10);
      i--;
  }
}

int readSensor(int sensor)
{
  long distHistory[10];
  long swapper = 0;

     for ( int i=0; i<10; i++ ) {
         distHistory[i] = analogRead(sensor) * 1.4;
         delay(10);
     }

     for(int out=0 ; out < 10; out++) {  // outer loop
       for(int in=0; in < 10; in++)  {  // inner loop  
         if( distHistory[in] > distHistory[in+1] ) {   // out of order?
           swapper = distHistory[in];
           distHistory [in] = distHistory[in+1];
           distHistory[in+1] = swapper;
         }
       }
     }

     return (long) distHistory[4];
}

void go()
{ 
  int newLSpeed = MAXSPEED;
  int newRSpeed = MAXSPEED+ERR;
  
  int wall = readSensor(ARight);
  
  if (checkRightTurn())
  {
    turnRight();
  }
  else if (checkLeftTurn())
  {
    turnLeft();
  }
  else
  {
    if (wall > DISTANCE + TRESHOLD)
    {
      newRSpeed += 10;
      newLSpeed = MAXSPEED;
    }
    else if (wall < DISTANCE - TRESHOLD)
    {
      newLSpeed += 10;
      newRSpeed = MAXSPEED;
    }
    else
    {
      newLSpeed = MAXSPEED;
      newRSpeed = MAXSPEED;
    } 
  }

  Serial.print("Left speed: ");
  Serial.print(newLSpeed);
  Serial.print(" --- ");
  Serial.print("Right speed: ");
  Serial.print(newRSpeed);
  Serial.print(" --- Distance: ");
  Serial.println(wall);

  digitalWrite(M1,LOW);
  digitalWrite(M2,LOW);
  analogWrite(E1,newLSpeed);
  analogWrite(E2,newRSpeed);
}

int checkRightTurn()
{
  int t1 = readSensor(ARight);
  delay(100);
  int t2 = readSensor(ARight);

  if (abs(t1-t2) > DIFF)
  {
    return 1;
  }
  return 0;
}

int checkLeftTurn()
{
  int t1 = readSensor(AFront);
  if (t1 > DISTANCE)
  {
    return 1;
  }
  return 0;
}

void turnLeft()
{
  while(readSensor(ARight) > DISTANCE)
  {
    analogWrite(E1,0);
    analogWrite(E2,MAXSPEED);
  }
}

void turnRight()
{
    analogWrite(E1,MAXSPEED);
    analogWrite(E2,MAXSPEED);

    for(int i=MAXSPEED; i >= 0; i--) 
    { 
      if (readSensor(ARight) > DISTANCE)
      {
        analogWrite(E2,i);
        analogWrite(E1, MAXSPEED);
        delay(100);
      }
      else
      {
        break;
      }
    }
}

void loop()
{
    accRobot();
    while(1)
    {
      go();
      delay(100);
    }
}
