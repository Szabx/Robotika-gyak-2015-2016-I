/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int e1 = 9;
int e2 = 10;
int m1 = 8;
int m2 = 11;
int m,e;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);  
  pinMode(m1, OUTPUT);  
  pinMode(m2, OUTPUT);  
}

void right(uint8_t e)
{
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  analogWrite(e1, e);
  analogWrite(e2, e);
}

void left(uint8_t e)
{
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  analogWrite(e1, e);
  analogWrite(e2, e);
}

void mv(int m,int e)
{
  digitalWrite(m1, m);
  digitalWrite(m2, m);
  analogWrite(e1, e);
  analogWrite(e2, e);
  delay(1000);
}

// the loop routine runs over and over again forever:
void loop() {

  mv(LOW, 120);
  for (int i=0; i<= 128; i++)
  {
    right(i);
    delay(10);
  }
  delay(500);
  mv(LOW, 120);
  for (int i=0; i<=128; i++)
  {
    left(i);
    delay(10);
  }
  delay(500);
}
