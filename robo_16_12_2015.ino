/*
 * A0 - front
 * A1 - left
 * A2 - right
 * 
 * E2(D10), M2(D11) - (servo) - right
 * E1(D9), M1(D8) - (servo) - left,    
 */
#include <SharpIR.h>
#include <Math.h>

#define BalSebesseg 9  //Bal kerek sebesseg
#define BalIrany 8  //Bal kered irany
#define JobbSebesseg 10 //Jobb kerek sebesseg
#define JobbIrany 11  //Jobb kerek irany
#define RANGE_MIN 1 //legkisebb tavolsag meres hatar
#define RANGE_MAX 100 //legnagyobb tavolsag meres hatar
#define MIN 20 //faltavolsag min
#define MAX 40 //faltavolsag max
#define GYORS 255 //nagyobbat fordulo kerek
#define LASSU 125 //kisebbet fordulo kerek
#define DEFAULT_SEBESSEG 155 //default sebesseg

#define KEREKATM 0.065 // Kerek atmeroje(m)
#define KEREKTAV 0.11 // Kerekek kozti tav(m)
#define PULSES_PER_REVOLUTION 48.0
// #define PI 3.14159

// Egy pozicio
struct position
{
  float x;        /* meter */
  float y;        /* meter */
  float theta;    /* radian (counterclockwise from x-axis) */
};

// Instance of position
struct position current_position;

// Szamolok
int BalCnt = 0;
int JobbCnt = 0;

// Odometria
int olvasasok = 0;

// Flag
boolean startInterrupt = false;

/*
 * Irany valtozo
 *  0 - elore
 *  1 - jobbra
 *  2 - hatra
 *  3 - balra
 */

int irany = 0;
int X = 0;
int Y = 0;
boolean elindult = false;

/* Parameterek
 * @1 - pin
 * @2 - hany meresbol szamoljon
 * @3 - hany %-a az egymast koveto mereseknek lesz valid
 * @4 - szenzor tipus
 */
SharpIR BALTAV(1, 20, 75, 20150);
SharpIR KOZEPTAV(0, 20, 75, 1080);
SharpIR JOBBTAV(2, 20, 75, 20150);

// Struct init
  void initialize_odometry()
  {
    current_position.x = 0.0;
    current_position.y = 0.0;
    current_position.theta = 0.0;
  }

void setup() {
  Serial.begin(115200);

  // Irany pin-ek inicializalasa
  pinMode(BalIrany, OUTPUT);   
  pinMode(JobbIrany, OUTPUT); 

  digitalWrite(BalIrany,LOW); 
  digitalWrite(JobbIrany,LOW);

  // Kerekfordulatszam-meres
  attachInterrupt(1, BalSzamolo, FALLING);
  attachInterrupt(0, JobbSzamolo, FALLING);
}

void loop() {
  go();
}

void go()
{
  int bt = tavolsag(BALTAV);
  int kt = tavolsag(KOZEPTAV);
  int jt = tavolsag(JOBBTAV);
  // 
  float MUL_COUNT;

  MUL_COUNT  = PI * KEREKATM / PULSES_PER_REVOLUTION;

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
    elore(DEFAULT_SEBESSEG);
  }

  odometer_thread();
//
  Serial.print("Theta: ");
  Serial.print(current_position.theta);

//  Serial.print("Left speed: ");
//  Serial.print(analogRead(LeftSpeed));
//  Serial.print(" --- ");
//  Serial.print("Right speed: ");
//  Serial.print(analogRead(RightSpeed));
//  Serial.print(" --- Distance: ");
//  Serial.print(BALTAV.distance());
//  Serial.print(" --- Front distance: ");
//  Serial.println(KOZEPTAV.distance());
//  Serial.print(" --- Irany: ");
//  Serial.print(irany);
//  Serial.print(" --- Balkerekfordulat: ");
//  Serial.print(BalCnt);
//  Serial.print(" --- Pulses per revolution: ");
//  Serial.print(BalCnt);
//  Serial.print(" --- jobbkerekfordulat: ");
//  Serial.println(JobbCnt);
}

void fordul(int sbs1, int sbs2)
{
  analogWrite(BalSebesseg, sbs1);
  analogWrite(JobbSebesseg, sbs2);
}

void hatra(int sebesseg)
{
  elore(-sebesseg);  
}

void elore(int sebesseg)
{
  analogWrite(BalSebesseg, sebesseg); 
  analogWrite(JobbSebesseg, sebesseg); 
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
     
  digitalWrite(BalIrany, LOW);
  digitalWrite(JobbIrany, HIGH);  
  // analogWrite(JobbSebesseg, 0);

  while(kozepTav < 40)
  {
    kozepTav = tavolsag(KOZEPTAV);
  }
  digitalWrite(BalIrany, LOW);
  digitalWrite(JobbIrany, LOW);
  stop();

//  irany++;
}

void BalSzamolo()
{
  if (startInterrupt)
  {
    BalCnt++;
  }
  if (BalCnt == 15)
  {
    X = Y = 15;
    elindult = true;
  }
  
  if (elindult)
  {
    switch(irany%4) {
      case 0:
        X++;
        break;
      case 1:
        Y++;
        break;
      case 2:
        X--;
        break;
      case 3:
        Y--;
        break;
    }  
  }
// Minden 10 fordulatnal terkepezunk
//  if (olvasasok%10 == 0){
//    UjPozicio();
//  }
//  olvasasok+=1;
}

void JobbSzamolo()
{
  if (startInterrupt)
  {
    JobbCnt++;
    irany++;
  }
}

void odometer_thread()
{
  float dist_left;
  float dist_right;
  int left_ticks;
  int right_ticks;
  float expr1;
  float cos_current;
  float sin_current;
  float right_minus_left;
  float MUL_COUNT;

  MUL_COUNT  = PI * KEREKATM / PULSES_PER_REVOLUTION;
  
  startInterrupt = false; //enable_interrupts(0);         /* Ensure we don't lose any odometer counts */
  left_ticks = BalCnt;
  right_ticks = JobbCnt;
  BalCnt = 0;
  JobbCnt = 0;
  startInterrupt = true;
//  enable_interrupts(1);
//  Serial.print(" -- PI -- ");
//  Serial.print(PI);
//  Serial.print(" --  -- ");

  dist_left  = (float)left_ticks * MUL_COUNT;
  dist_right = (float)right_ticks * MUL_COUNT;

  cos_current = cos(current_position.theta);
  sin_current = sin(current_position.theta);
  Serial.print(current_position.x);
  Serial.print(" -- vs -- ");
  Serial.println(current_position.y);

  if (left_ticks == right_ticks)
  {
    /* Moving in a straight line */
    current_position.x += dist_left * cos_current;
    current_position.y += dist_left * sin_current;
  }
  else
  {
    /* Moving in an arc */
    expr1 = KEREKTAV * (dist_right + dist_left)
            / 2.0 / (dist_right - dist_left);

    right_minus_left = dist_right - dist_left;

    current_position.x += expr1 * (sin(right_minus_left /
                          KEREKTAV + current_position.theta) - sin_current);

    current_position.y -= expr1 * (cos(right_minus_left /
                          KEREKTAV + current_position.theta) - cos_current);

    /* Calculate new orientation */
    current_position.theta += right_minus_left / KEREKTAV;

    /* Keep in the range -PI to +PI */
    while(current_position.theta > PI)
      current_position.theta -= (2.0*PI);
    while(current_position.theta < -PI) 
      current_position.theta += (2.0*PI); 
      
    delay(100); 
  } 
} 

//void UjPozicio()
//{
//}
