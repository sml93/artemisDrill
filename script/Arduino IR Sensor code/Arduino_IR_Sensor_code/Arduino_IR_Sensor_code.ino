//code by YaSh.

int sen = A0;
const float wid = 0.012;   //adjust   the width of the fan blade according to your needs.
const float rad = 0.045;    //adjust the radius of the point of detection in the fan blade.
const float   konst = 6.2832;  //  2*pi.
float time_1;
float time_2;
float vel;
float   diff;
float tnet;
float rpm;
void setup() 
{
  Serial.begin(9600);
   pinMode(sen,INPUT);
  Serial.print(" \
 please start the motor at least 3   seconds prior.\
");
  delay(3000);
}
void loop()
{
  if(analogRead(sen)<950)
   {
    time_1 = millis();
    delay(30);
  }
  if(analogRead(sen)>950)
   {
    time_2 = millis();
    diff = (time_2-time_1);         
    vel   = wid/diff;                      //rotation velocity
    tnet = (konst*rad)/vel;               //time = (2*pi*radius)/velocity.
    rpm = (60000)/tnet;                //   time in ms to minutes and then to rpm conversion step.
  }
  Serial.print("\
   The rpm is : "); 
  Serial.println( int(rpm) );

}