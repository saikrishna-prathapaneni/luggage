#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <MechaQMC5883.h>
#include <Wire.h>
#include <math.h>

#define trigPinOne D4
#define echoPinOne D5
#define trigPinTwo D2
#define echoPinTwo D3
#define trigPinThree D6
#define echoPinThree D7
#define motAR D8
#define motAL D9
#define motBR D10
#define motBL D11


#define earthRadiusKm 6371000
#define DECLINATION_ANGLE 0.25
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

char auth[] = "343788959dba4eb68a03f1215e55e949";
char ssid[] = "Shivatejan4g";
char pass[] = "@shivatejan1";

TinyGPSPlus gps;
SoftwareSerial ss(12, 14);
bool turnOn = false;
MechaQMC5883 qmc;

float heading , bearing;
double dis;


struct LOC
{
  double lat,lon;
};
struct LOC phone , bot;

void checkObstacles(int &d1,int &d2,int &d3)
{
int t1,t2,t3;

digitalWrite(trigPinOne, LOW);
delayMicroseconds(2);
digitalWrite(trigPinOne, HIGH);
delayMicroseconds(10);
digitalWrite(trigPinOne,LOW);
t1=pulseIn(echoPinOne,HIGH);

delayMicroseconds(2);
digitalWrite(echoPinOne,LOW);

digitalWrite(trigPinTwo,LOW);
delayMicroseconds(2);
digitalWrite(trigPinTwo,HIGH);
delayMicroseconds(10);
digitalWrite(trigPinTwo,LOW);
t2=pulseIn(echoPinTwo,HIGH);

delayMicroseconds(2);
digitalWrite(echoPinTwo,LOW);


digitalWrite(trigPinThree,LOW);
delayMicroseconds(2);
digitalWrite(trigPinThree,HIGH);
delayMicroseconds(10);
digitalWrite(trigPinThree,LOW);
t3=pulseIn(echoPinThree,HIGH);
delayMicroseconds(2);
digitalWrite(echoPinThree,LOW);

d1 = t1 /58;
d2 = t2 /58;
d3 = t3 /58;
}




int noObstacles()
{
  int distance_one,distance_two,distance_three;
  checkObstacles(distance_one,distance_two,distance_three);
  if(distance_one < 20 && distance_one < distance_two && distance_one < distance_three)
    {
        while(distance_one > 20 )
        {
          turnRight();
          checkObstacles(distance_one,distance_two,distance_three);
        }
        stop();
    }
  if(distance_two < 20 && distance_two < distance_one && distance_two < distance_three)
      {
        while(distance_two > 20 )
        {
          turnLeft();
          checkObstacles(distance_one,distance_two,distance_three);
        }
        stop();
      }
  if(distance_three < 20 && distance_three < distance_one && distance_three < distance_two)
  {
      if(distance_one < distance_two)
      {
          while(distance_three > 20 )
          {
            turnRight();
            checkObstacles(distance_one,distance_two,distance_three);
          }
          stop();
      }
      else
      {
          while(distance_three > 20 )
          {
            turnLeft();
            checkObstacles(distance_one,distance_two,distance_three);
          }
          stop();
      }
  }
  return 1;
}



void setup()
{
  Serial.begin(9600);
  ss.begin(9600);
  Wire.begin();
  Blynk.begin(auth, ssid, pass);
  qmc.init();
}

void stop()
{
  digitalWrite(motAR,LOW);
  digitalWrite(motAL,LOW);
  digitalWrite(motBR,LOW);
  digitalWrite(motBL,LOW);
}

void turnRight()
{
  digitalWrite(motAR,HIGH);
  digitalWrite(motAL,LOW);
  digitalWrite(motBR,LOW);
  digitalWrite(motBL,HIGH);
}

void turnLeft()
{
  digitalWrite(motBR,HIGH);
  digitalWrite(motBL,LOW);
  digitalWrite(motAR,LOW);
  digitalWrite(motAL,HIGH);
}
void goStraight()
{

  if(noObstacles())
        {
          digitalWrite(motBR,HIGH);
          digitalWrite(motBL,LOW);
          digitalWrite(motAR,HIGH);
          digitalWrite(motAL,LOW);
        }
}


double deg2rad(double deg) {
  return (deg * M_PI / 180);
}



void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    bot.lat = gps.location.lat();
    bot.lon = gps.location.lng();
    Serial.print(bot.lat,7);
    Serial.print(F(","));
    Serial.print(bot.lon,7 );
  }
  else
  {
    bot.lat = 0;
    bot.lon = 0;
    Serial.print(F("INVALID"));
  }
}



void loc()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    bot.lat = 0;
    bot.lon = 0;
    while(true);
  }

}


void driveTo()
{
      loc();
        if(bot.lat!=0 && bot.lon!=0 && turnOn)
        {
            int time = 18;
            do
            {
                loc();
                geoDist();
                geoAngle();
                head();
                float turn = bearing - heading;
                Serial.println("distance = ");
                Serial.print(dis);
                Serial.print("   ,  turn = ");
                Serial.print(turn);
                goTo(turn);
                time--;
            } while(dis > 4 && time && turnOn);
            stop();
        }
}


void goTo(float turning)
{
  float t = turning;
  while (t < -180)  t += 360;
  while (t > 180 )   t -= 360;

  Serial.print("actual turn = ");
  Serial.println(t);

  while(heading != bearing)
  {
      head();
      geoAngle();
      if(t > 0)
          turnRight();
      else if(t < 0)
        turnLeft();

  }
  stop();
  while(heading == bearing)
  {
    head();
    geoAngle();
    goStraight();
  }
  stop();
}





void head()
{

  int x,y,z;
  qmc.read(&x,&y,&z);
  float Heading = atan2(y,x);
  if(Heading < 0)
    Heading += 2*3.142857142857143;
  if(Heading > 2*3.142857142857143)
    Heading -= 2*3.142857142857143;
    Heading *= 57.2957795;

  while(Heading < -180)
    Heading += 360;
  while (Heading > 180)
    Heading -= 360;
    heading = Heading;
}

void geoAngle()
{
  double y = sin(phone.lon-bot.lon) * cos(phone.lat);
  double x = cos(bot.lat)*sin(phone.lat) - sin(bot.lat)*cos(phone.lat)*cos(phone.lon-bot.lon);
  bearing =  atan2(y, x) * RADTODEG;
  Serial.print("bear :");
  Serial.println(bearing);
}





void geoDist()
{
  double lat1r, lon1r, lat2r, lon2r, u, v;

  lat1r  = deg2rad(bot.lat);
  lon1r = deg2rad(bot.lon);
  lat2r = deg2rad(phone.lat);
  lon2r = deg2rad(phone.lon);
  u = sin((lat2r - lat1r)/2);
  v = sin((lon2r - lon1r)/2);

  dis = 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));


  Serial.print("distance :");
  Serial.println(dis);

}





BLYNK_WRITE(V1)
{
  GpsParam phoneGPS(param);
  Serial.println("phone gps!!");
  Serial.println("Location: ");
  phone.lat = phoneGPS.getLat() ;
  phone.lon = phoneGPS.getLon();
  Serial.print(phone.lat,7);
  Serial.print("  ,  ");
  Serial.print(phone.lon,7);
  Serial.println("");

}
BLYNK_WRITE(V2)
{
    turnOn = !turnOn;
    Serial.println(turnOn);
}
void loop()
{

  driveTo();
  Blynk.virtualWrite(V3,dis);
  Blynk.virtualWrite(V4,heading);
  Blynk.virtualWrite(V5,bearing);

  Blynk.run();

}
