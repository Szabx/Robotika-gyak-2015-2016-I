int sensorpin = 0;                 // analog pin used to connect the sharp sensor
int val = 0;                 // variable to store the values from sensor(initially zero)

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int e1 = 9;
int e2 = 10;
int m1 = 8;
int m2 = 11;
int m,e;

void setup()
{
  // initialize the digital pin as an output.
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);  
  pinMode(m1, OUTPUT);  
  pinMode(m2, OUTPUT);  
  Serial.begin(9600);               // starts the serial monitor
}
 
void loop()
{
  val = analogRead(sensorpin);       // reads the value of the sharp sensor
  Serial.println(val);            // prints the value of the sensor to the serial monitor
  delay(100);                    // wait for this much time before printing next value
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  digitalWrite(e1, HIGH);
  digitalWrite(e2, HIGH);
}
