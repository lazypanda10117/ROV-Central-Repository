#include <Wire.h> 
#include <OneWire.h>
#include <SparkFun_MS5803_I2C.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 5

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
MS5803 sensor(ADDRESS_HIGH);


double temp;
String tempString;
double pressure_abs,pressure_baseline;
double depth;
String depthString;
double base_altitude = 1655.0; // Altitude of SparkFun's HQ in Boulder, CO. in (m)


void setup() {
  Serial.begin(1200);
  Serial.println("Temperature Sensor Initialized");
  sensors.begin();
  sensors.setResolution(0,9);
    sensor.reset();
    sensor.begin();
    
    pressure_baseline = sensor.getPressure(ADC_4096);
    
  Wire.begin(); 
}
void loop() {
  sensors.requestTemperatures();
  temp = sensors.getTempCByIndex(0);
  tempString = String(temp,2);
  pressure_abs = sensor.getPressure(ADC_4096);
  depth = ((pressure_abs-1013) * 0.033417)*0.305;
  depthString = String(depth,2);
  char tA[11]; 
  for(int i=0; i<11; i++){
    if(i<5){
      tA[i] = tempString.charAt(i);
    }else if(i==5){
      tA[i] = 'p';
    }else{
      tA[i] = depthString.charAt(i-6);
    }
  }
    Serial.println("temp array");
    for(int i=0; i<5; i++){
      Serial.print(tA[i]);
    }
    
    Serial.print("Depth = ");
    for(int i=6; i<11; i++){
      Serial.print(tA[i]);
    } 
    Serial.println("m");

  Wire.beginTransmission(9); // transmit to device #9
  Wire.write(tA,11);
  Serial.println("\nSending Temperature Data");

  Wire.endTransmission();    // stop transmitting
  Serial.println("\n ended");

  delay(200);
}
 
