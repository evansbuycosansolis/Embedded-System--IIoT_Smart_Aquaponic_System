#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>


#define ONE_WIRE_BUS 5
#define DO_PIN A1
#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution
#define TWO_POINT_CALIBRATION 0
#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function
#define CAL1_V (1600) //mv
#define CAL1_T (25)   //℃
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃
#define SensorPin 0          //pH meter Analog output to Arduino Analog Input 0
#define TdsSensorPin A1
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

 float Celcius=0;
 float Fahrenheit=0;
 
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};
const byte nsum=10;
int humidityPin=A3;
int Thgm20Pin=A4;
unsigned int sensorValue2 = 0;  // variable to store the value coming from the sensor
unsigned int sensorValue3 = 0;  // variable to store the value coming from the sensor
uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10],temp;


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


//===========================================================================

void setup()
{
  Serial.begin(115200);
  sensors.begin();
  Wire.begin();
//Configure HDC1080
Wire.beginTransmission(0x40);
Wire.write(0x02);
Wire.write(0x90);
Wire.write(0x00);
Wire.endTransmission();
//Delay for Startup of HDC1080
delay(20);
  pinMode(13,OUTPUT);  
  Serial.println("Ready");    //Test the serial monitor
  pinMode(TdsSensorPin,INPUT);
}

//===========================================================================



void loop()
{
  _DissolvedO();
  DS18b20(void);
  HDC1080();
   HSM20G();
   _pH_Sensor();
   _TDS_();
  }

//===========================================================================

void _DissolvedO()
{
  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;

  Serial.print("Temperaturet:\t" + String(Temperaturet) + "\t");
  Serial.print("ADC RAW:\t" + String(ADC_Raw) + "\t");
  Serial.print("ADC Voltage:\t" + String(ADC_Voltage) + "\t");
  Serial.println("DO:\t" + String(readDO(ADC_Voltage, Temperaturet)) + "\t");

  delay(1000);
}


void DS18b20(void)
{ 
  sensors.requestTemperatures(); 
  Celcius=sensors.getTempCByIndex(0);
  Fahrenheit=sensors.toFahrenheit(Celcius);
  Serial.print(" C  ");
  Serial.print(Celcius);
  Serial.print(" F  ");
  Serial.println(Fahrenheit);
  delay(1000);
}


void HDC1080()
{
double temperature;
double humidity;
humidity = readSensor(&temperature);

//Print the current temperature to the right of the label
Serial.print("Temperature: ");
Serial.println(temperature);


//Print the current humidity to the right of the label
Serial.print("Humidity: ");
Serial.println(humidity);

//Wait 1 second for the next reading
delay(1000);
}



double readSensor(double* temperature)
{
//holds 2 bytes of data from I2C Line
uint8_t Byte[4];
//holds the total contents of the temp register
uint16_t temp;
//holds the total contents of the humidity register
uint16_t humidity;
//Point to device 0x40 (Address for HDC1080)
Wire.beginTransmission(0x40);
//Point to register 0x00 (Temperature Register)
Wire.write(0x00);
//Relinquish master control of I2C line
//pointing to the temp register triggers a conversion
Wire.endTransmission();
//delay to allow for sufficient conversion time
delay(20);
//Request four bytes from registers
Wire.requestFrom(0x40, 4);
delay(1);
//If the 4 bytes were returned sucessfully
if (4 <= Wire.available())
{
//take reading
//Byte[0] holds upper byte of temp reading
Byte[0] = Wire.read();
//Byte[1] holds lower byte of temp reading
Byte[1] = Wire.read();
//Byte[3] holds upper byte of humidity reading
Byte[3] = Wire.read();
//Byte[4] holds lower byte of humidity reading
Byte[4] = Wire.read();
//Combine the two bytes to make one 16 bit int
temp = (((unsigned int)Byte[0] <<8 | Byte[1]));
//Temp(C) = reading/(2^16)*165(C) – 40(C)
*temperature = (double)(temp)/(65536)*165-40;
//Combine the two bytes to make one 16 bit int
humidity = (((unsigned int)Byte[3] <<8 | Byte[4]));
//Humidity(%) = reading/(2^16)*100%
return (double)(humidity)/(65536)*100;
}
}

void HSM20G()
{
    for (byte i=0;i<nsum;i++)
  {    
    sensorValue2 += analogRead(humidityPin);    
    sensorValue3 += analogRead(Thgm20Pin); 
  } 

     
  int sensorValue2Avg = sensorValue2/nsum;
  int humidity= 0.5151*sensorValue2Avg-12.0;

  int sensorValue3Avg=sensorValue3/nsum;
  float Vt=(float) sensorValue3Avg*5.0/1023.0;
  float R=(5.0-Vt)*10.0/Vt;
  float temperature=281.583*pow(1.0230,(1.0/R))*pow(R,-0.1227)-173.4; 
  
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print(temperature);
  Serial.println(" deg C");

  delay(1000);

  sensorValue2=0;
  sensorValue3=0;
  delay(2000);
}

void _pH_Sensor()
{
  for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(SensorPin);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=3.5*phValue;                      //convert the millivolt into pH value
  Serial.print("    pH:");  
  Serial.print(phValue,2);
  Serial.println(" ");
  digitalWrite(13, HIGH);       
  delay(800);
  digitalWrite(13, LOW); 

}

void _TDS_()
{
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
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
