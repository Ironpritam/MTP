#include <Adafruit_SCD30.h>
#include <sps30.h>
#include "DFRobot_OxygenSensor.h"
#include <MQUnifiedsensor.h>
#include "DFRobot_MICS.h"

// Define sensor instances
Adafruit_SCD30 scd30;
DFRobot_OxygenSensor oxygen;
MQUnifiedsensor MQ131("STM32", 5, 10, A0, "MQ-131, O2 & SPS30 sensor");
DFRobot_MICS_ADC mics(A2, 10);

#define COLLECT_NUMBER  10
#define Oxygen_IICAddress ADDRESS_3
#define RatioMQ131CleanAir 15
#define CALIBRATION_TIME 0

#define RL 10000   // Load resistance for MICS5524 (typically 10kΩ)
#define V_REF 3.3  // Reference voltage (STM32 typically 3.3V)

// MICS5524 calibration value
float R0_MICS;
                        // |for reducing gasses CO:     CH4(methane):     C2H5OH(Ethanol):   |for oxidizing gasses   NO2(Nitrogen Dioxide):      O3 (Ozone):        
float A_MICS = 600;     // |                    600     700               3.5                |                       1.2                         3.6
float B_MICS = -1.2;    // |                    -1.2    -1.5              -1.1               |                       -1.1                        −1.1

// Sampling interval (3 minutes = 180,000 milliseconds)
const unsigned long sampleInterval = 15000;
unsigned long previousMillis = 0;


void setupSCD30() {
  if (!scd30.begin()) {
    // Serial.println("Failed to find SCD30 chip");
    while (1) { delay(10); }
  }
  // Serial.println("SCD30 Found!");
}

void setupSPS30() {
  int16_t ret;
  uint8_t auto_clean_days = 4;
  sensirion_i2c_init();
  
  while (sps30_probe() != 0) {
    //Serial.println("SPS sensor probing failed");
    delay(500);
  }
  ret = sps30_start_measurement();
}

void setupOxygenSensor() {
  if (!oxygen.begin(Oxygen_IICAddress)) {
    //Serial.println("Error: Oxygen Sensor not connected!");
    while (1);
  }
  //Serial.println("Oxygen Sensor successfully connected!");
}

void setupMQ131() {
  MQ131.setRegressionMethod(1);
  MQ131.setA(23.943);
  MQ131.setB(-1.11);
  MQ131.init();
  
  //Serial.println("Calibrating MQ131. Please wait.");
  float calcR0 = 0;
  
  for (int i = 1; i <= 10; i++) {
    MQ131.update();
    calcR0 += MQ131.calibrate(RatioMQ131CleanAir);
    //Serial.print(".");
    delay(1000);
  }
  
  MQ131.setR0(calcR0 / 10);
  //Serial.println(" done!");

  if (isinf(calcR0) || calcR0 == 0) {
    //Serial.println("MQ131 Calibration Warning: Check wiring and supply.");
    while (1);
  }
  //MQ131.serialDebug();
}

void setupMICS() {
  while (!mics.begin()) {
    //Serial.println("NO Devices !");
    delay(1000);
  }
  //Serial.println("MICS-5524 sensor connected successfully!");

  if (mics.getPowerState() == SLEEP_MODE) {
    mics.wakeUpMode();
    //Serial.println("Wake up MICS sensor success!");
  } else {
    //Serial.println("The sensor is already in wake-up mode");
  }

  while (!mics.warmUpTime(CALIBRATION_TIME)) {
    //Serial.println("Please wait until the warm-up time is over!");
    delay(1000);
  }

  // Calibrate the MICS sensor
  int numReadings = 10;
  float rsSum = 0;
  
  //Serial.println("Calibrating MICS5524 sensor. Please ensure it is in clean air...");
  for (int i = 0; i < numReadings; i++) {
    int adcValue = mics.getADCData(0); // OX_MODE
    float voltage = (adcValue * V_REF) / 4095; // STM32 12-bit ADC
    float Rs = ((V_REF * RL) / voltage) - RL;
    rsSum += Rs;
    delay(1000); // Delay between readings
  }
  
  R0_MICS = rsSum / numReadings;
  //Serial.print("MICS5524 Calibration complete! R0: ");
  //Serial.println(R0_MICS);
}


void logData(float temp, float humidity, float CO2, float pm1, float pm2_5, float pm4, float pm10, float O2, float O3, float MICS) {
  // Format the data as a CSV row: timestamp, temp, humidity, CO2, PM1, PM2.5, PM4, PM10, O2, O3, MICS
  // Serial.print(timestamp); Serial.print(",");
  Serial.print(temp); Serial.print(",");
  Serial.print(humidity); Serial.print(",");
  Serial.print(CO2); Serial.print(",");
  Serial.print(pm1); Serial.print(",");
  Serial.print(pm2_5); Serial.print(",");
  Serial.print(pm4); Serial.print(",");
  Serial.print(pm10); Serial.print(",");
  Serial.print(O2); Serial.print(",");
  Serial.print(O3); Serial.print(",");
  Serial.println(MICS);
}

void readSensors() {

  // SCD30 (Temperature, Humidity, CO2)
  float temp=1,humidity=0,CO2=1;
  int i = 0;
  for (i = 0; i < 5; i++) {
    if (scd30.dataReady() && scd30.read() ) {
      temp = scd30.temperature;
      humidity = scd30.relative_humidity;
      CO2 = scd30.CO2;
      break; // Exit the loop when data is ready
    } else {
      delay(1000); // Wait 1 second before trying again
    }
  }
  

  // SPS30 (PM1, PM2.5, PM4, PM10)
  struct sps30_measurement m;
  uint16_t data_ready;
  int16_t ret;
  i=0;
  do {
    ret = sps30_read_data_ready(&data_ready);
    if (ret >= 0 && data_ready) {
      break;
    }
    delay(1000);
  } while (1 && i<15 );
  
  sps30_read_measurement(&m);
  // m.mc_XXX : Mass/Weight concentration ,unit : µg/m³ (meter's cube)
  // m.nc_XXX : Number concentration , unit : particles/cm³ (cm cube)
  float pm1 = m.mc_1p0;    // x/1000 to convert from ppb to ppm.
  float pm2_5 = m.mc_2p5;
  float pm4 = m.mc_4p0;
  float pm10 = m.mc_10p0;
  

  // Oxygen Sensor (O2)
  float O2 = oxygen.getOxygenData(COLLECT_NUMBER);
  

  // MQ131 (O3)
  MQ131.update();
  // can print Ozone (O3) concentration directly using "Serial.println(readSensorR0Rs)""
  float O3 = MQ131.readSensorR0Rs();
  

  // MICS (Gas Concentration)
  int adcValue = mics.getADCData(0);  // 0 for OX_MODE Reads the oxidizing mode (O2 and NO2 gases),  1 for RED_MODE Reads the reducing mode (CO, CH4, and other reducing gases)
  // float voltage = (adcValue * V_REF) / 4095;  // Convert ADC value to voltage
  // float Rs = ((V_REF * RL) / voltage) - RL;  // Calculate Rs (sensor resistance)
  // float ratio = Rs / R0_MICS;
  // // Formula for gas concentration based on datasheet (adjust according to gas type)
  // float MICS = A_MICS * pow(ratio, B_MICS); // Example for CO, adjust per gas type

  // Log data in CSV format
  logData(temp, humidity, CO2, pm1, pm2_5, pm4, pm10, O2, O3, adcValue);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  setupSCD30();
  setupSPS30();
  setupOxygenSensor();
  setupMQ131();
  setupMICS();
  delay(5000);
  // Print CSV headers
  Serial.println("Temperature(in oC),Humidity(in RH),CO2(in ppm),PN1(in µg/m³),PN2.5,PN4,PN10,Oxygen(%),Ozone(analog_value),MICS_Concentration(analog_value)");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check if the sampling interval has passed
  
  if (currentMillis - previousMillis >= sampleInterval) {
    previousMillis = currentMillis;
    
    // Read sensor data and log it
    
    readSensors();
  }
}
