#include <SPI.h>
#include <MFRC522.h>
#include <ros.h>
#include <barc/Light.h>
#include <barc/Echo.h>
#include <barc/RFID.h>

#define trigPin 2
#define echoPin 3

const int analogInPin = A6;

constexpr uint8_t RST_PIN = 9;
constexpr uint8_t SS_PIN = 10;

MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class


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
barc::RFID rf;


ros::NodeHandle nh;

ros::Publisher pub_light("light", &light);
ros::Publisher pub_echo("echo", &echo);
ros::Publisher pub_rfid("rfid", &rf);


void setup() {
  // initialize serial communications at 9600 bps:
  //Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  SPI.begin();      // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522 

  // Start ROS node
  nh.initNode();

  // Publish and subscribe to topics
  nh.advertise(pub_light);
  nh.advertise(pub_echo);
  nh.advertise(pub_rfid);
}

void loop() {

  if ( rfid.PICC_IsNewCardPresent()){
      if(! rfid.PICC_ReadCardSerial()){
          rf.id = -2;
      }
      else
      {

          // Verify if the NUID has been readed
          //printHex(rfid.uid.uidByte, rfid.uid.size);
          //printDec(rfid.uid.uidByte, rfid.uid.size);

          rf.id = rfid.uid.uidByte[1]; //UIDs differ in second byte

      }
      // Halt PICC
      rfid.PICC_HaltA();

      // Stop encryption on PCD
      rfid.PCD_StopCrypto1();

  }
  else
  {
      rf.id = -1;
  }
  pub_rfid.publish(&rf);

  light.light = getLight();
  echo.distance = getDistance();

  pub_light.publish(&light);
  pub_echo.publish(&echo);

  nh.spinOnce();
  delay(100);
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

int getUID(){
  // Look for new cards
  if ( ! rfid.PICC_IsNewCardPresent())
    return -1;

  // Verify if the NUID has been readed
  if ( ! rfid.PICC_ReadCardSerial())
    return -1;
  //printHex(rfid.uid.uidByte, rfid.uid.size);
  //printDec(rfid.uid.uidByte, rfid.uid.size);

  int id = rfid.uid.uidByte[1]; //UIDs differ in second byte

  // Halt PICC
  rfid.PICC_HaltA();

  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();

  return id;
}
