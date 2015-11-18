// DFRobot test program

#define E1 9
#define M1 8
#define E2 10
#define M2 11

void setup()
{
 Serial.begin(9600);
pinMode(13,OUTPUT);
pinMode(M1,OUTPUT);
pinMode(M2,OUTPUT);
digitalWrite(M1,LOW);
digitalWrite(M2,LOW);

}

void loop(){
int value;
for(value=0;value<=255;value+=5){
analogWrite(E1,value);
analogWrite(E2,value);
delay(50);
digitalWrite(13,LOW);
delay(50);
digitalWrite(13,HIGH);
Serial.println(value);
}

}
