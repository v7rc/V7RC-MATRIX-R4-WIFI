/*
  WiFi Web Server LED Blink

  A simple web server that lets you blink an LED via the web.
  This sketch will create a new access point (with no password).
  It will then launch a new server and print out the IP address
  to the Serial Monitor. From there, you can open that address in a web browser
  to turn on and off the LED on pin 13.

  If the IP address of your board is yourAddress:
    http://yourAddress/H turns the LED on
    http://yourAddress/L turns it off

  created 25 Nov 2012
  by Tom Igoe
  adapted to WiFi AP by Adafruit

  Find the full UNO R4 WiFi Network documentation here:
  https://docs.arduino.cc/tutorials/uno-r4-wifi/wifi-examples#access-point
 */

#include "WiFiS3.h"
#include "arduino_secrets.h" 
#include <Servo.h>
#include <MatrixMiniR4.h>

#define SERVO_DEFAULT_VALUE 1500
#define SERVO_DEFAULT_MAX_VALUE 2000
#define SERVO_DEFAULT_MIN_VALUE 1000
#define PACKET_LENGTH 20
#define numOfServo 2

#define loopPeriod 30
#define debugMode false

// Servo Pin,  目前先用8組;
const int SERVO_Pin[] = {2, 7, 3, 8, 4, 9, 5, 11};
Servo robotServo[numOfServo];

int count = 0;               //計數用

int servValue[numOfServo];            // 目前的伺服馬達數值
int oldServValue[numOfServo];            // 舊的的伺服馬達數值
int failSafeServValue[numOfServo];    // 各個伺服馬達FailSafe設定
int servoMAXValue[numOfServo];        // 每個Servo最大的數值
int servoMINValue[numOfServo];        // 每個Servo最小的數值
int bytes[PACKET_LENGTH];
int receiveServoValue[numOfServo];

int dcMotorPinA[] = {12, 10};     // DC motor A
int dcMotorPinB[] = {13, 17};     // DC motor B

long startProcessTime = 0;
long endProcessTime = 0;

// 測試Servo相關數據
boolean ifTestMode = false;   // 是否進入測試模式
boolean ifAddValue = false;   // 是否增加資料
int servoMoveStepValue = 80;  // 測試用的速度

#define LOST_SIGNAL_MAX_TIME 500 // 最大失去信號時間;

int currentLostSignalTime = 0;

int accelPWMValue = SERVO_DEFAULT_VALUE;   // 需要控制油門的的PWM;
int accelPWMChannel = 1;      // Channel0, 1, 2, 3, 4, 5, 6....
bool isControlAccelerator = false;          // 是否需要限制油門;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key index number (needed only for WEP)

int led =  LED_BUILTIN;
int status = WL_IDLE_STATUS;

unsigned int localPort = 6188;      // local port to listen on

char packetBuffer[256]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged\n";       // a string to send back

// 設定UDP Server
WiFiUDP Udp;

void setup() {
  MiniR4.begin();
  MiniR4.PWR.setBattCell(2);  // 18650x2, two-cell (2S)
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  MiniR4.M1.setBrake(true); // Motor brake
  MiniR4.M2.setBrake(true);
  MiniR4.M3.setBrake(true);
  MiniR4.M4.setBrake(true);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Access Point Web Server");

  pinMode(led, OUTPUT);      // set the LED pin mode

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  WiFi.config(IPAddress(192,168,1,1));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  delay(5000);

  // you're connected now, so print out the status
  printWiFiStatus();

  Udp.begin(localPort);
}

int dataIndex = 0;
byte dataBytes[PACKET_LENGTH];
String thisPacket;

void loop() {
  
  // // compare the previous status to the current status
  // if (status != WiFi.status()) {
  //   // it has changed update the variable
  //   status = WiFi.status();

  //   if (status == WL_AP_CONNECTED) {
  //     // a device has connected to the AP
  //     Serial.println("Device connected to AP");
  //   } else {
  //     // a device has disconnected from the AP, and we are back in listening mode
  //     Serial.println("Device disconnected from AP");
  //   }
  // }

  // if there's data available, read a packet

  int startTime = millis();

  int packetSize = Udp.parsePacket();
  if (packetSize) {

    if(debugMode) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remoteIp = Udp.remoteIP();
      Serial.print(remoteIp);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());
    }
    
    // read the packet into packetBuffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    if(debugMode) {
      Serial.println("Contents:");
      Serial.println(packetBuffer);
    }

    if(packetSize == 20) {

      // 取得Payload;
      thisPacket = String(packetBuffer);

      processRCString(thisPacket);
      // 表示有收到命令;

    }
    
    // send a reply, to the IP address and port that sent us the packet we received
    // Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    // Udp.write(ReplyBuffer);
    // Udp.endPacket();
  }

  int spentTime = millis() - startTime;

  if(loopPeriod - spentTime > 0) {
    delay(loopPeriod - spentTime);
  }
  
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("START UDP SERVER!");
  Serial.println(ip);

}

// 目前只要大於等於16Bytes, 並且在最後有個井字好結尾，那就是合理的command;
void processRCString(String command) {

  int commandLength = command.length();

  if (commandLength > 15) {

    if (command.charAt(commandLength - 1) != '#') {  // 表示結尾不是預設的結果;

      return;

    }

  } else {

    return;
  }

  Serial.println(command);


  if (command.indexOf("SRV") > -1 || command.indexOf("SS4") > -1  ) {  // 表示伺服馬達操作;

    // Serial.println("接收到伺服馬達命令;");

    int i = 3;
    int servoIndex = 0;

    while (i < commandLength - 1) {     // 解碼;

      if (i + 3 < commandLength) {

        String singleCommand = command.substring(i, i + 4);

        // Serial.println(singleCommand);

        if (servoIndex < numOfServo) {

          receiveServoValue[servoIndex] = singleCommand.toInt();

          
           if (receiveServoValue[servoIndex] != oldServValue[servoIndex]) {

              if(receiveServoValue[servoIndex] > SERVO_DEFAULT_MAX_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MAX_VALUE;
              if(receiveServoValue[servoIndex] < SERVO_DEFAULT_MIN_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MIN_VALUE;
              oldServValue[servoIndex] = receiveServoValue[servoIndex];

//               int thisAngle = map(receiveServoValue[servoIndex], 544, 2400, 0, 180);
//               robotServo[servoIndex].write(thisAngle);
              // robotServo[servoIndex].writeMicroseconds(receiveServoValue[servoIndex]);
           }
          
          servoIndex ++;

        }


      }

      i = i + 4;

    }

    processMatrixR4DCMotor(receiveServoValue[0], 0);
    processMatrixR4DCMotor(receiveServoValue[1], 1);

    // processDCMotor(receiveServoValue[1], dcMotorPinA);
    // processDCMotor(receiveServoValue[0], dcMotorPinB);

    // processServoCommand(receiveServoValue);     // 處理伺服馬達;

  } else if (command.indexOf("SRT") > -1 ) {  // 表示伺服馬達操作;

    // Serial.println("接收到伺服馬達命令;");

    int i = 3;
    int servoIndex = 0;

    while (i < commandLength - 1) {     // 解碼;

      if (i + 3 < commandLength) {

        String singleCommand = command.substring(i, i + 4);

        // Serial.println(singleCommand);

        if (servoIndex < numOfServo) {

          receiveServoValue[servoIndex] = singleCommand.toInt();

          
           if (receiveServoValue[servoIndex] != oldServValue[servoIndex]) {

              if(receiveServoValue[servoIndex] > SERVO_DEFAULT_MAX_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MAX_VALUE;
              if(receiveServoValue[servoIndex] < SERVO_DEFAULT_MIN_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MIN_VALUE;
              oldServValue[servoIndex] = receiveServoValue[servoIndex];

//               int thisAngle = map(receiveServoValue[servoIndex], 544, 2400, 0, 180);
//               robotServo[servoIndex].write(thisAngle);
              //crobotServo[servoIndex].writeMicroseconds(receiveServoValue[servoIndex]);
           }
          
          servoIndex ++;

        }

        

      }

      i = i + 4;

    }

    // 這裡要處理坦克的訊號
    int tankServo1 = 1500;
    int tankServo0 = 1500;

    if(receiveServoValue[1] >= 1500) {

      int duration = receiveServoValue[1] - 1500;
      
      tankServo1 = tankServo1 + duration;
      tankServo0 = tankServo0 - duration;
       
    } else {

      int duration = 1500 - receiveServoValue[1]  ;

      tankServo1 = tankServo1 - duration;
      tankServo0 = tankServo0 + duration;
       
    }

    if(receiveServoValue[0] >= 1500) {

       int duration = receiveServoValue[0] - 1500;
      
      tankServo1 = tankServo1 + duration;
      tankServo0 = tankServo0 + duration;
    } else {

      int duration = 1500 - receiveServoValue[0]  ;
      
      tankServo1 = tankServo1 - duration;
      tankServo0 = tankServo0 - duration;
    }

//    Serial.print("tankServo0:");
//    Serial.println(tankServo0);
//
//    Serial.print("tankServo1:");
//    Serial.println(tankServo1);

    if(tankServo0 < 1000) {
      tankServo0 = 1000;
    }

    if(tankServo0 > 2000) {
      tankServo0 = 2000;
    }

     if(tankServo1 < 1000) {
      tankServo1 = 1000;
    }

    if(tankServo1 > 2000) {
      tankServo1 = 2000;
    }

    receiveServoValue[0] = tankServo0;
    receiveServoValue[1] = tankServo1;

//    Serial.print("servo 0:");
//    Serial.println(receiveServoValue[0]);

    // processDCMotor(receiveServoValue[0], dcMotorPinA);
    // processDCMotor(receiveServoValue[1], dcMotorPinB);

    processMatrixR4DCMotor(receiveServoValue[0], 0);
    processMatrixR4DCMotor(receiveServoValue[1], 1);

    // 傳給ProcessServoCommand發送訊號;
    processServoCommand(receiveServoValue);     // 處理伺服馬達;

  } else if(command.indexOf("SR2") > -1) {
    // Serial.println("接收到第二組伺服馬達命令;");

    int i = 3;
    int servoIndex = 4;

    while (i < commandLength - 1) {     // 解碼;

      if (i + 3 < commandLength) {

        String singleCommand = command.substring(i, i + 4);

        // Serial.println(singleCommand);

        if (servoIndex < numOfServo) {

          receiveServoValue[servoIndex] = singleCommand.toInt();

          
           if (receiveServoValue[servoIndex] != oldServValue[servoIndex]) {

              if(receiveServoValue[servoIndex] > SERVO_DEFAULT_MAX_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MAX_VALUE;
              if(receiveServoValue[servoIndex] < SERVO_DEFAULT_MIN_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MIN_VALUE;
              oldServValue[servoIndex] = receiveServoValue[servoIndex];

//               int thisAngle = map(receiveServoValue[servoIndex], 544, 2400, 0, 180);
//               robotServo[servoIndex].write(thisAngle);
              // robotServo[servoIndex].writeMicroseconds(receiveServoValue[servoIndex]);
           }
          
          servoIndex ++;

        }

        

      }

      i = i + 4;

    }
  } else if(command.indexOf("SS8") > -1) {
    // Serial.println("接收到第二組伺服馬達命令;");

    int i = 3;
    int servoIndex = 4;

    while (i < commandLength - 1) {     // 解碼;

      if (i + 3 < commandLength) {

        String singleCommand = command.substring(i, i + 2);

        // Serial.println(singleCommand);

        if (servoIndex < numOfServo) {

          receiveServoValue[servoIndex] = singleCommand.toInt();

          
           if (receiveServoValue[servoIndex] != oldServValue[servoIndex]) {

              if(receiveServoValue[servoIndex] > SERVO_DEFAULT_MAX_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MAX_VALUE;
              if(receiveServoValue[servoIndex] < SERVO_DEFAULT_MIN_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MIN_VALUE;
              oldServValue[servoIndex] = receiveServoValue[servoIndex];

               int pwmValue = map(receiveServoValue[servoIndex], 0, 99, 544, 2400);
               // robotServo[servoIndex].write(thisAngle);
               // robotServo[servoIndex].writeMicroseconds(pwmValue);
           }
          
          servoIndex ++;

        }

      }

      i = i + 2;

    }
  }

}


//將命令發送至servo
void V7RCCommand(int servoValue1, int servoValue2, int servoValue3, int servoValue4, int servoValue5, int servoValue6, int servoValue7, int servoValue8)
{

  // Serial.println( servoValue1 );
  //       Serial.println("servoAngle[0]: "+String(servoAngle[0])+"servoAngle[1]: " +String(servoAngle[1]));

  if (servoValue1 != oldServValue[0]) {

    if (servoValue1 > servoMAXValue[0]) {
      servoValue1 = servoMAXValue[0];
    }
    else if (servoValue1 < servoMINValue[0]) {
      servoValue1 = servoMINValue[0];
    }

    oldServValue[0] = servoValue1;
    // robotServo[0].write(servoValue1);
    //
    //delay(5);
  }

  if (servoValue2 != oldServValue[1]) {


    if (servoValue2 > servoMAXValue[1]) {
      servoValue2 = servoMAXValue[1];
    }
    else if (servoValue2 < servoMINValue[1]) {
      servoValue2 = servoMINValue[1];
    }

    oldServValue[1] = servoValue2;
    // robotServo[1].write(servoValue2);
    //servo_channel2.write(servoValue2);
    //delay(5);
  }

  if (servoValue3 != oldServValue[2]) {

    if (servoValue3 > servoMAXValue[2]) {
      servoValue3 = servoMAXValue[2];
    }
    else if (servoValue3 < servoMINValue[2]) {
      servoValue3 = servoMINValue[2];
    }

    oldServValue[2] = servoValue3;
    // robotServo[2].write(servoValue2);
    //servo_channel3.write(servoValue3);
    //delay(5);
  }

  if (servoValue4 != oldServValue[3]) {

    if (servoValue4 > servoMAXValue[3]) {
      servoValue4 = servoMAXValue[3];
    }
    else if (servoValue4 < servoMINValue[3]) {
      servoValue4 = servoMINValue[3];
    }

    oldServValue[3] = servoValue4;
    // robotServo[3].write(servoValue4);
    //delay(5);
  }

  if (servoValue5 != oldServValue[4]) {

    if (servoValue5 > servoMAXValue[4]) {
      servoValue5 = servoMAXValue[4];
    }
    else if (servoValue5 < servoMINValue[4]) {
      servoValue5 = servoMINValue[4];
    }

    oldServValue[4] = servoValue5;
    // robotServo[4].write(servoValue5);
    //delay(5);
  }

  if (servoValue6 != oldServValue[5]) {

    if (servoValue6 > servoMAXValue[5]) {
      servoValue6 = servoMAXValue[5];
    }
    else if (servoValue6 < servoMINValue[5]) {
      servoValue6 = servoMINValue[5];
    }

    oldServValue[5] = servoValue6;
    // robotServo[5].write(servoValue6);
    //delay(5);
  }

  if (servoValue7 != oldServValue[6]) {

    if (servoValue7 > servoMAXValue[6]) {
      servoValue7 = servoMAXValue[6];
    }
    else if (servoValue7 < servoMINValue[6]) {
      servoValue7 = servoMINValue[6];
    }

    oldServValue[6] = servoValue7;
    // robotServo[6].write(servoValue7);
    //delay(5);
  }

  if (servoValue8 != oldServValue[7]) {

    if (servoValue8 > servoMAXValue[7]) {
      servoValue8 = servoMAXValue[7];
    }
    else if (servoValue8 < servoMINValue[7]) {
      servoValue8 = servoMINValue[7];
    }

    oldServValue[7] = servoValue8;
    // robotServo[7].write(servoValue8);
    //delay(5);
  }

  //  servo_channel1.write(servoValue2);
  //  servo_channel2.write(servoValue2);
  //  servo_channel3.write(servoValue3);
  //  servo_channel4.write(servoValue4);
  //  servo_channel5.write(servoValue5);
  //  servo_channel6.write(servoValue6);
  // servoPosition(SERVO_Pin1, servoAngle[0]);

}

void processServoCommand(int servoValue[]) {

  int thisIndex = 0;
  while (thisIndex < numOfServo && thisIndex < sizeof(servoValue)) {

    robotServo[thisIndex].writeMicroseconds(servoValue[thisIndex]);

    thisIndex ++;
  }

}

void processMatrixR4DCMotor(int pwmValue, int motorNO) {

  int power =  map(pwmValue, 1000, 2000, -100 , 100); 

  if(motorNO == 0) {
    MiniR4.M1.setPower(power);
  } else if(motorNO == 1) {
     MiniR4.M2.setPower(power);
  }else if(motorNO == 2) {
     MiniR4.M3.setPower(power);
  }else if(motorNO == 3) {
     MiniR4.M4.setPower(power);\
  }
}

void processDCMotor(int pwmValue, int dcMotor[]) {

  // if(pwmValue == 1500) {
    
  //   digitalWrite(dcMotor[0], LOW);
  //   analogWrite(dcMotor[1], 0);
  
  // } else if(pwmValue > 1500) {
  //   int power = map(pwmValue, 1500, 2000, 0 , 255); 
  //   digitalWrite(dcMotor[0], LOW);
  //   analogWrite(dcMotor[1], power);
  // } else {
  //   int power = map(pwmValue, 1500, 1000, 255 , 0); 
  //   digitalWrite(dcMotor[0], HIGH);
  //   analogWrite(dcMotor[1], power);
  // }
}
