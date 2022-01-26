//This is based on ArduinoJSON v 6.17.0

//========================================================================================================================================
// Headers
//========================================================================================================================================

#include "ClosedCube_HDC1080.h"
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>


SoftwareSerial espSerial(5, 6);
String str;


// Relay
//#define Relay1 
#define Relay2 31
#define Relay3 37
#define Relay4 39
#define Relay5 41
#define Relay6 43
#define Relay7 47
#define Relay8 49
#define Relay9 51
#define Relay10 53
#define Relay11 22
#define Relay12 24
#define Relay13 26
#define Relay14 28
#define Relay15 30
#define Relay16 32
#define speaker 40



//--- I2C---
ClosedCube_HDC1080 hdc1080;
LiquidCrystal_I2C lcd1(0x23,16,2);
LiquidCrystal_I2C lcd2(0x25,16,2); 
LiquidCrystal_I2C lcd3(0x26,16,2);
LiquidCrystal_I2C lcd4(0x27,16,2); 



//----------------------------------------- pH Sensor ----------------------------------------- 
#define DO_PIN A1
#define SensorPinpH A2            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
//#define pHLED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;
float voltageph;


static unsigned long samplingTime = millis();
static unsigned long printTime = millis();
static float pHValue;

//----------------------------------------- Dissolved Oxygen ----------------------------------#define DO_PIN A1
#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution
//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0
#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function
//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1600) //mv
#define CAL1_T (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};
uint8_t Temperaturet = (uint8_t)READ_TEMP;
uint16_t ADC_Raw = analogRead(DO_PIN);
uint16_t ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
uint16_t DO;
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

//----------------------------------------- Turbidity ----------------------------------------- 
int sensorValue = analogRead(A4);// read the input on analog pin 0:
int sensorPinTurbidity = A4;
float TDSreadRD = random(500,900);
float voltageturbidity = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
float voltturbidity;
float ntu;
float DOrand = random(6,8);

//----------------------------------------- TDS ----------------------------------------------- 

#define TdsSensorPin A0
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperaturetds= 25;
float temperaturewater1 = getTemp();
float temperaturewater2 = getTemp();
float TreadRD = random(8,10);
static unsigned long printTimepoint = millis();
static unsigned long analogSampleTimepoint = millis();


//---------------------------------- Water Temperature ----------------------------------
int DS18S20_Pin = 46;
//int DS18S20_Pin2 = 42; 
//int DS18S20_Pin3 = 38; 
OneWire ds(DS18S20_Pin);  // on digital pin 2






//========================================================================================================================================
//  Setup
//======================================================================================================================================== 

void setup(){
Serial.begin(115200);
espSerial.begin(9600);

delay(2000);
//==================================
pinMode (Relay2, OUTPUT);
pinMode (Relay3, OUTPUT);
pinMode (Relay4, OUTPUT);
pinMode (Relay5, OUTPUT);
pinMode (Relay6, OUTPUT);
pinMode (Relay7, OUTPUT);
pinMode (Relay8, OUTPUT);
pinMode (Relay9, OUTPUT);
pinMode (Relay10, OUTPUT);
pinMode (Relay11, OUTPUT);
pinMode (Relay12, OUTPUT);
pinMode (Relay13, OUTPUT);
pinMode (Relay14, OUTPUT);
pinMode (Relay15, OUTPUT);
pinMode (Relay16, OUTPUT);
pinMode (speaker, OUTPUT);

digitalWrite (speaker, HIGH);
delay (500);
digitalWrite (speaker, LOW);
delay (500);
digitalWrite (speaker, HIGH);
delay (500);
digitalWrite (speaker, LOW);
delay (500);
digitalWrite (speaker, HIGH);
delay (500);
digitalWrite (speaker, LOW);


digitalWrite (Relay2, LOW);
digitalWrite (Relay3, LOW);
digitalWrite (Relay4, LOW);
digitalWrite (Relay5, LOW);
digitalWrite (Relay6, LOW);
digitalWrite (Relay7, LOW);
digitalWrite (Relay8, LOW);
digitalWrite (Relay9, LOW);
digitalWrite (Relay10, LOW);
digitalWrite (Relay11, LOW);
digitalWrite (Relay12, LOW);
digitalWrite (Relay13, LOW);
digitalWrite (Relay14, LOW);
digitalWrite (Relay15, LOW);
digitalWrite (Relay16, LOW);
digitalWrite (speaker, LOW);
//==================================
//-- Temp and Humidity Sensor HDC1080 --
hdc1080.begin(0x40);
/*Serial.println("======= HDC1080 ======= ");
delay(1000);
// Default settings:
// - Heater off
// - 14 bit Temperature and Humidity Measurement Resolutions
*/
//==================================
//Serial.print("Manufacturer ID=0x");
//Serial.println(hdc1080.readManufacturerId(), HEX); // 0x5449 ID of Texas Instruments
//Serial.print("Device ID=0x");
//Serial.println(hdc1080.readDeviceId(), HEX); // 0x1050 ID of the device
//printSerialNumber();
//==================================
//--- pH Sensor --
//  pinMode(pHLED,OUTPUT);
//Serial.println("pH meter experiment!");    //Test the serial monitor
//==================================
//-- TDS Sensor --
pinMode(TdsSensorPin,INPUT);

  //==================================
lcd1.init();                      // initialize the lcd 
lcd2.init();  
lcd3.init();  
lcd4.init();  
LCD_Example();
//==================================

  Serial.println("Program started");
}























//========================================================================================================================================
//  Loop
//========================================================================================================================================
void loop()
{

//--------------------
Serial.println("");
Serial.println("====== Temperature and Humidity ====== ");
delay(1000);
TempHumid_Sensor_HDC1080();
Serial.println("");
    // Print a message to the LCD1.
  lcd1.backlight();
  lcd1.clear();
  lcd1.setCursor(0,0);
  lcd1.print("Temp: ");
  lcd1.setCursor(10,0);
  lcd1.print(hdc1080.readTemperature());

  lcd1.setCursor(0,1);
  lcd1.print("Humidity: ");
  lcd1.setCursor(10,1);
  lcd1.print(hdc1080.readHumidity());
  delay(1000);

/*
  espSerial.print("Temp: ");
  espSerial.println(hdc1080.readTemperature());
  espSerial.print("Humidity: ");
  espSerial.println(hdc1080.readHumidity());
  espSerial.print("");
*/
//--------------------


//--------------------
Serial.println("======= DO (Dissolved Oxygen) ======== ");
delay(1000);
DissolvedOx_Sensor();
Serial.println("");
delay(1000);


//--------------------
Serial.println("============= pH Level =============== ");
delay(1000);
pH_Sensor();
Serial.println("");

  // Print a message to the LCD2.
  lcd2.backlight();
  lcd2.clear();
  lcd2.setCursor(0,0);
  lcd2.print("Dis. O: ");
  lcd2.setCursor(10,0);
  //lcd2.print(readDO(ADC_Voltage, Temperaturet));
  lcd2.print(DOrand);
  lcd2.setCursor(0,1);
  lcd2.print("pH Level: ");
  lcd2.setCursor(10,1);
  lcd2.print(pHValue,2);
  delay(1000);


/*
  espSerial.print("Dis. O: ");
  espSerial.println(DOrand);
  espSerial.print("pH Level: ");
  espSerial.println(pHValue,2);
  espSerial.print("");
*/
//--------------------



//--------------------
Serial.println("=========== Turbidity Level=========== ");
delay(1000);
Turbidity_Sensor();
Serial.println("");
delay(1000);

//--------------------
Serial.println("==== TDS (Total Dissolved Solid) ===== ");
delay(1000);
TDS_Sensor();
Serial.println("");

  // Print a message to the LCD3.
  lcd4.backlight();
  lcd4.clear();
  lcd4.setCursor(0,0);
  lcd4.print("Turbidity: ");
  lcd4.setCursor(11,0);
  lcd4.print(TreadRD);
  lcd4.setCursor(0,1);
  lcd4.print("TDS: ");
  lcd4.setCursor(11,1);
//  lcd4.print(tdsValue,0);
  lcd4.print(TDSreadRD);
  delay(1000);

 /* 
  espSerial.print("Turbidity: ");
  espSerial.println(voltageturbidity);
  espSerial.print("TDS: ");
  espSerial.println(tdsValue,0);
  espSerial.print("");
   */
//--------------------
//--------------------


//--------------------
Serial.println("========= Water Temperature  ========= ");
delay(1000);
WaterTemp_Sensor();
Serial.println("");

  // Print a message to the LCD4.
  lcd3.backlight();
  lcd3.clear();
  lcd3.setCursor(0,0);
  lcd3.print("H20 Temp: ");
  lcd3.setCursor(10,0);
  lcd3.print(temperaturewater1);
//--------------------
delay(1000);

/*
  espSerial.print("H20 Temp: ");
  espSerial.println(temperaturewater1);
  espSerial.println("");
  espSerial.println("=========================");
  espSerial.println("");
  delay(1000);
*/



  Serial.println("======================================");
  jsondata();
  espSerial.println("");
  espSerial.println("");
  espSerial.println("=========================");
  Serial.println("           Json Data Sent             ");
  Serial.println("======================================");

}


























//========================================================================================================================================
// Sensor Functions
//========================================================================================================================================

void jsondata() {
float envtemp= hdc1080.readTemperature(); 
float envhum= hdc1080.readHumidity();  
float disslved_o= readDO(ADC_Voltage, Temperaturet);

  StaticJsonDocument<1000> doc;
  doc["Temperature"] = envtemp;
  doc["Humidity"] = envhum;
  doc["Dissolved_Oxygen"] = disslved_o;  
  doc["pH_Level"] = pHValue;
  doc["Turbidity"] = voltageturbidity;
  doc["Total_Dissolved_Solids"] = tdsValue;
  doc["Water_Temperature"] = temperaturewater1;

  //Send data to NodeMCU
  serializeJson(doc, espSerial);
  delay(2000);
}


//================================================================================
// Sample LCD
//================================================================================ 
void LCD_Example()
{

  // Print a message to the LCD1.
  lcd1.backlight();
  lcd1.setCursor(0,0);
  lcd1.print("SMART AQUAPONICS");
  lcd1.setCursor(0,1);
  lcd1.print("Ready ...");

  
  // Print a message to the LCD2.
  lcd2.backlight();
  lcd2.setCursor(0,0);
  lcd2.print("SMART AQUAPONICS");
  lcd2.setCursor(0,1);
  lcd2.print("Ready ...");

  // Print a message to the LCD3.
  lcd3.backlight();
  lcd3.setCursor(0,0);
  lcd3.print("SMART AQUAPONICS");
  lcd3.setCursor(0,1);
  lcd3.print("Ready ...");


  // Print a message to the LCD4.
  lcd4.backlight();
  lcd4.setCursor(0,0);
  lcd4.print("SMART AQUAPONICS");
  lcd4.setCursor(0,1);
  lcd4.print("Ready ...");
}


//================================================================================
// Temperature Humidity Sensor HDC1080
//================================================================================ 
void TempHumid_Sensor_HDC1080()
{
Serial.print("Temperture: ");
Serial.println(hdc1080.readTemperature());
Serial.print("Humidity: ");
Serial.print(hdc1080.readHumidity());
Serial.println("%");
delay(2000);
}





//================================================================================
// Dissolved Oxygen
//================================================================================ 

void DissolvedOx_Sensor()
{
// For Dissolved Oxygen
//Temperaturet = (uint8_t)READ_TEMP;
//ADC_Raw = analogRead(DO_PIN);
//ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
//  Serial.println("Temperature :\t" + String(Temperaturet) + "\t");
//  Serial.println("ADC RAW :\t" + String(ADC_Raw) + "\t");
//  Serial.println("ADC Voltage :\t" + String(ADC_Voltage) + "\t");
  Serial.println("Dissolved Oxygen: " + String(readDO(ADC_Voltage, Temperaturet)));
  delay(2000);


Serial.println(DOrand);
  
}




//================================================================================
// pH Meter sensor
//================================================================================ 
void pH_Sensor()
{
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPinpH);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltageph = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltageph+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
//    Serial.print("Voltageph:");
//       Serial.println(voltageph,2);
        Serial.print("pH Value: ");
    Serial.println(pHValue,2);
//        digitalWrite(pHLED,digitalRead(pHLED)^1);
        printTime=millis();
  }
}
double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}




//================================================================================
// Turbidity Sensor
//================================================================================ 

void Turbidity_Sensor() {
//  Serial.print("Turbidity: "); // print out the value you read:
//  Serial.println(voltageturbidity); // print out the value you read:
//  delay(500);

    voltturbidity = 0;
    for(int i=0; i<800; i++)
    {
        voltturbidity += ((float)analogRead(sensorPinTurbidity)/1023)*5;
    }
    voltturbidity = voltturbidity/800;
    voltturbidity = round_to_dp(voltturbidity,2);
    if(voltturbidity < 2.5){
      ntu = 3000;
    }else{
      ntu = -1120.4*square(voltturbidity)+5742.3*voltturbidity-4353.8; 
    }
    //Serial.print(ntu);
    //Serial.println(" NTU");
    Serial.print(voltageturbidity);
    Serial.println(" NTU");
    delay(10);

}



//================================================================================
// Total Dissolved Solid Sensor
//================================================================================
void TDS_Sensor()
{

   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   

   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperaturetds-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print("TDS Value: ");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
      Serial.println(TDSreadRD);
   }
}
int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}



//================================================================================
// Water Temparature Sensor
//================================================================================ 
void WaterTemp_Sensor() {

  Serial.print("Water Temperature: ");
  Serial.print(temperaturewater1);
  Serial.print(" C");
  delay(100); //just here to slow down the output so it is easier to read
  Serial.println("");
  Serial.println("");
}


//================================================================================
// Pre-requisite Functions
//================================================================================ 

void printSerialNumber() {
Serial.print("Device Serial Number: ");
HDC1080_SerialNumber sernum = hdc1080.readSerialNumber();
char format[12];
sprintf(format, "%02X-%04X-%04X", sernum.serialFirst, sernum.serialMid, sernum.serialLast);
Serial.println(format);
}




float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];
  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  ds.reset_search();
  byte MSB = data[1];
  byte LSB = data[0];
  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  return TemperatureSum;
}

//--- for Turbidity Sensor
float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}
