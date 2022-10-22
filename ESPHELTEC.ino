#include "arduino_secrets.h"
#include <TTN_esp32.h>
#include "TTN_CayenneLPP.h"
#include "MQ7.h"
#define RXD2 16
#define TXD2 17
#define CO_PIN 37
#define VOLTAGE 5

// init MQ7 device
MQ7 mq7(CO_PIN, VOLTAGE);

const char* devAddr = DEVADD; // Change to TTN Device Address
const char* nwkSKey = NWKEY; // Change to TTN Network Session Key
const char* appSKey = APPKEY;// Change to TTN Application Session Key

TTN_esp32 ttn ;
TTN_CayenneLPP lpp;



String var1 = "", var2 = "", var3 = "", var4 = "", var5 = "";
int temp = 0, hum = 0, pr = 0;
float ws = 0;

/************************Hardware Related Macros************************************/
#define         MG_PIN                       (36)     //define which analog input channel you are going to use
#define         BOOL_PIN                     (2)
#define         DC_GAIN                      (8.5)   //define the DC gain of amplifier

/***********************Software Related Macros************************************/
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interval(in milisecond) between each samples in
                                                     //normal operation

/**********************Application Related Macros**********************************/
//These two values differ from sensor to sensor. user should derermine this value.
#define         ZERO_POINT_VOLTAGE           (0.235) //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define         REACTION_VOLTGAE             (0.030) //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2

/*****************************Globals***********************************************/
float           CO2Curve[3]  =  {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};
                                                     //two points are taken from the curve.
                                                     //with these two points, a line is formed which is
                                                     //"approximately equivalent" to the original curve.
                                                     //data format:{ x, y, slope}; point1: (lg400, 0.324), point2: (lg4000, 0.280)
                                                     //slope = ( reaction voltage ) / (log400 –log1000)
void message(const uint8_t* payload, size_t size, int rssi)
{
  Serial.println("-- MESSAGE");
  Serial.print("Received " + String(size) + " bytes RSSI= " + String(rssi) + "dB");

  for (int i = 0; i < size; i++)
  {
    Serial.print(" " + String(payload[i]));
    // Serial.write(payload[i]);
  }

  Serial.println();
}
void setup()
{
   pinMode(BOOL_PIN, INPUT);                        //set pin to input
    digitalWrite(BOOL_PIN, HIGH);                    //turn on pullup resistors
   Serial.print("MG-811 Demostration\n");
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Calibrating MQ7");
  mq7.calibrate();    // calculates R0
  Serial.println("Calibration done!");
  ttn.begin();
  ttn.onMessage(message); // declare callback function when is downlink from server
  ttn.personalize(devAddr, nwkSKey, appSKey);
  ttn.showStatus();
}

void loop() {
   int percentage;
    float volts;

    volts = MGRead(MG_PIN);
    Serial.print( "SEN0159:" );
    Serial.print(volts);
    Serial.print( "V   " );

    percentage = MGGetPercentage(volts,CO2Curve);
    Serial.print("CO2:");
    if (percentage <400) {
        //Serial.print( "<400" );
       Serial.print(percentage);
    } else {
        Serial.print(percentage);
    }

    Serial.print( "ppm" );
    Serial.print("\n");

    if (digitalRead(BOOL_PIN) ){
        Serial.print( "=====BOOL is HIGH======" );
    } else {
        Serial.print( "=====BOOL is LOW======" );
    }

    Serial.print("\n");
    delay(500);
    
  while (Serial2.available() > 0) {
    var1 = Serial2.readStringUntil(','); // ws
    Serial2.read();delay(1000);
    var2 = Serial2.readStringUntil(','); //wind direction
    Serial2.read();delay(1000);
    var3 = Serial2.readStringUntil(','); // temp
    Serial2.read(); delay(1000);
    var4 = Serial2.readStringUntil(','); // hum
    Serial2.read(); delay(1000);
    var5 = Serial2.readStringUntil('\n'); // pression
    ws = atoi(var1.c_str()); temp = atoi(var3.c_str()); hum = atoi(var4.c_str()); pr = atoi(var5.c_str());
  //  Serial.print("VAR3==="); Serial.print(var3);
  
    Serial.print("temp :"); Serial.print(temp);
    Serial.print(",ws :"); Serial.print(ws);
    Serial.print("wd :"); Serial.println(var2);
    Serial.print("CO PPM = "); Serial.println(mq7.readPpm());
    Serial.print("CO2 PPM = "); Serial.println(percentage);

    delay(3000);
  }
  lpp.reset();
  lpp.addTemperature(1, temp);
  lpp.addBarometricPressure(2, pr);
  lpp.addRelativeHumidity(3, hum);
  sendingWD(var2);
  lpp.addAnalogInput(5, mq7.readPpm());
  lpp.addLuminosity(6, percentage);
  lpp.addAnalogInput(7, ws);
  // Send it off
  ttn.sendBytes(lpp.getBuffer(), lpp.getSize());
 // delay(1000);
}
void sendingWD(String var) {
  if (var2 == "N") {
    lpp.addAnalogInput(4, 0);
  }
  else if (var2 == "NE") {
    lpp.addAnalogInput(4, 45);
  }
  else if (var2 == "E") {
    lpp.addAnalogInput(4, 90);
  }
  else if (var2 == "SE") {
    lpp.addAnalogInput(4, 135);
  }
  else if (var2 == "S") {
    lpp.addAnalogInput(4, 180);
  }
  else if (var2 == "SW") {
    lpp.addAnalogInput(4, 225);
  }
  else if (var2 == "W") {
    lpp.addAnalogInput(4, 270);
  }
  else if (var2 == "NW") {
    lpp.addAnalogInput(4, 315);
  } else {
    lpp.addAnalogInput(4, 111);
  }
}

/*****************************  MGRead *********************************************
Input:   mg_pin - analog channel
Output:  output of SEN-000007
Remarks: This function reads the output of SEN-000007
************************************************************************************/
float MGRead(int mg_pin)
{
    int i;
    float v=0;

    for (i=0;i<READ_SAMPLE_TIMES;i++) {
        v += analogRead(mg_pin);
        delay(READ_SAMPLE_INTERVAL);
    }
    v = (v/READ_SAMPLE_TIMES) *5/4095 ;
    return v;
}

/*****************************  MQGetPercentage **********************************
Input:   volts   - SEN-000007 output measured in volts
         pcurve  - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(MG-811 output) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
int  MGGetPercentage(float volts, float *pcurve)
{
   if ((volts/DC_GAIN )>=ZERO_POINT_VOLTAGE) {
      return pow(10, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
   } else {
      return pow(10, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
   }
}
