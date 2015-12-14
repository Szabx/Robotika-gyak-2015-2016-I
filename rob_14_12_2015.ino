/*
 * A0 - front
 * A1 - left
 * A2 - right
 * 
 * E2(D10), M2(D11) - (servo) - right
 * E1(D9), M1(D8) - (servo) - left,    
 */
#include <SharpIR.h>

#define LeftSpeed 9  //Bal kerek sebesseg
#define LeftDir 8  //Bal kered irany
#define RightSpeed 10 //Jobb kerek sebesseg
#define RightDir 11  //Jobb kerek irany
#define RANGE_MIN 1; //legkisebb tavolsag meres hatar
#define RANGE_MAX 100; //legnagyobb tavolsag meres hatar
#define MIN 30; //faltavolsag min
#define MAX 50; //faltavolsag max
#define GYORS 255; //nagyobbat fordulo kerek
#define LASSU 125; //kisebbet fordulo kerek
#define DEFAULT_SEBESSEG 155; //default sebesseg

/* Parameterek
 * @1 - pin
 * @2 - hany meresbol szamoljon
 * @3 - hany %-a az egymast koveto mereseknek lesz valid
 * @4 - szenzor tipus
 */
SharpIR BALTAV(1, 20, 75, 20150);
SharpIR KOZEPTAV(0, 20, 75, 1080);
SharpIR JOBBTAV(2, 20, 75, 20150);

void setup() {
  Serial.begin(115200);
 
  pinMode(LeftDir, OUTPUT);   
  pinMode(RightDir, OUTPUT); 

  digitalWrite(LeftDir,LOW); 
  digitalWrite(RightDir,LOW);

}

void loop() {
  go();

}

void go()
{
  int bt = tavolsag(BALTAV);
  int kt = tavolsag(KOZEPTAV);
  int jt = tavolsag(JOBBTAV);

  if(bt > 15 && jt < 40)
  {
    jobbraFordul();
  }

  if(bt < MIN)
  {
    fordul(GYORS,LASSU); //jobbra
  }
  else if(bt > MAX)
  {
    fordul(LASSU,GYORS); //balra
  }
  else
  {
    forward(DEFAULT_SEBESSEG);
  }

//  Serial.print("Left speed: ");
//  Serial.print(analogRead(LeftSpeed));
//  Serial.print(" --- ");
//  Serial.print("Right speed: ");
//  Serial.print(analogRead(RightSpeed));
  Serial.print(" --- Distance: ");
  Serial.print(DIS_L.distance());
  Serial.print(" --- Front distance: ");
  Serial.println(DIS_M.distance());
}

void fordul(int spd1, int spd2)
{
  analogWrite(LeftSpeed, spd1);
  analogWrite(RightSpeed, spd2);
}

void hatra(int sebesseg)
{
  elore(-sebesseg);  
}

void elore(int sebesseg)
{
  analogWrite(LeftSpeed, sebesseg); 
  analogWrite(RightSpeed, sebesseg); 
}

void stop()
{
    elore(0);
}

int tavolsag(SharpIR szenzor)
{
  int tav = szenzor.distance();
  if(tav < RANGE_MIN)
  {
    tav = RANGE_MIN;
  }
  if(tav > RANGE_MAX)
  {
    tav = RANGE_MAX;  
  }
  return tav;
}

void jobbraFordul()
{
  int kozepTav = tavolsag(KOZEPTAV);
     
  digitalWrite(LeftDir, LOW);
  digitalWrite(RightDir, HIGH);  
  

  while(kozepTav < 40)
  {
    kozepTav = tavolsag(KOZEPTAV);
  }
  digitalWrite(LeftDir, LOW);
  digitalWrite(RightDir, LOW);
  stop();
}
