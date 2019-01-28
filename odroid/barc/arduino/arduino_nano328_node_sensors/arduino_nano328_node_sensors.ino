#include <ros.h>
#include <barc/Light.h>
#include <barc/Echo.h>

#define trigPin 2
#define echoPin 3

const int analogInPin = A6;


/*
typedef struct {
  byte size;  //Number of bytes in the UID. 4, 7 or 10.
  byte uidByte[10];
  byte sak;
} Uid
*/

// Global message variables
barc::Light light;
barc::Echo echo;


ros::NodeHandle nh;

ros::Publisher pub_light("light", &light);
ros::Publisher pub_echo("echo", &echo);


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Start ROS node
  nh.initNode();

  // Publish and subscribe to topics
  nh.advertise(pub_light);
  nh.advertise(pub_echo);
}

void loop() {
  light.light = getLight();
  pub_light.publish(&light);

  echo.distance = getDistance();
  if(echo.distance < 150)
      pub_echo.publish(&echo);

  nh.spinOnce();
  delay(200);
}

int getLight(){
  int sensorValue = 0;        // value read from the pot
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  return map(sensorValue, 0, 1023, 0, 255);
}

float getDistance(){
  long duration;
  float distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  //  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;

  return distance;
}
