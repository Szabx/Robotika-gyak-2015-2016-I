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

int AFront = 0, ALeft = 1, ARight = 2; 

// int countbal = 0,countjobb = 0;

int DISTANCE = 300;
int TRESHOLD = 20;

void setup()
{
//  attachInterrupt(1, erzekelobal, CHANGE);
//  attachInterrupt(0, erzekelojobb, CHANGE);
  Serial.begin(9600);
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
  int newRSpeed = MAXSPEED+15;
  
  int wall = readSensor(ARight);
  if (wall < DISTANCE)
  {
    newLSpeed += 10;
  }
  else if (wall > DISTANCE)
  {
    newRSpeed += + 10;
  }
  else
  {
    newLSpeed = MAXSPEED+20;
    newRSpeed = MAXSPEED;
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

void loop()
{
//    delay(1000);
    accRobot();
    while(1)
    {
      go();
      delay(500);
    }
}

//void erzekelobal()
//{
//  countbal++;
//}
//
//void erzekelojobb()
//{
//  countjobb++;
//}
