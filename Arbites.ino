#include <Wire.h>
#include <DFRobot_MLX90614.h>
#include "EMGFilters.h"
#include "DFRobot_BloodOxygen_S.h"

#define MLX90614_I2C_ADDR 0x5A
#define SensorInputPin A0   //sensor input pin number
#define I2C_ADDRESS 0x57

unsigned long threshold = 0;  // threshold: Relaxed baseline values.(threshold=0:in the calibration process)
unsigned long EMG_num = 0;      // EMG_num: The number of statistical signals

DFRobot_MLX90614_I2C tempSensor(MLX90614_I2C_ADDR, &Wire);
DFRobot_BloodOxygen_S_I2C MAX30102(&Wire, I2C_ADDRESS);
EMGFilters myFilter;

SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_500HZ;
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;


void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // Initialize MLX90614
    while (NO_ERR != tempSensor.begin()) {
        Serial.println("MLX90614: Communication failed");
        delay(3000);
    }
    Serial.println("MLX90614: Initialized");
    tempSensor.setEmissivityCorrectionCoefficient(1.0);
    tempSensor.setMeasuredParameters(tempSensor.eIIR100, tempSensor.eFIR1024);

    // Initialize MAX30102
    while (!MAX30102.begin()) {
        Serial.println("MAX30102: Initialization failed");
        delay(1000);
    }
    Serial.println("MAX30102: Initialized");
    MAX30102.sensorStartCollect();
    
    // Initialize EMG Filter
    myFilter.init(sampleRate, humFreq, true, true, true);
}

void loop() {
    // Read MLX90614 Temperatures
    float ambientTemp = tempSensor.getAmbientTempCelsius();
    float objectTemp = tempSensor.getObjectTempCelsius();
    Serial.print("Ambient Temp: "); Serial.print(ambientTemp); Serial.println(" °C");
    Serial.print("Object Temp: "); Serial.print(objectTemp); Serial.println(" °C");

    // Read MAX30102 Heart Rate & SpO2
    MAX30102.getHeartbeatSPO2();
    Serial.print("SPO2: "); Serial.print(MAX30102._sHeartbeatSPO2.SPO2); Serial.println("%");
    Serial.print("Heart Rate: "); Serial.print(MAX30102._sHeartbeatSPO2.Heartbeat); Serial.println(" BPM");
    Serial.print("Board Temp: "); Serial.print(MAX30102.getTemperature_C()); Serial.println(" °C");
    
    Serial.println("---------------------");
    //delay(1000); // Delay to allow sensor processing

     // Read EMG Sensor Data
    int data = analogRead(SensorInputPin);
    int dataAfterFilter = myFilter.update(data);  // filter processing
    int envelope = sq(dataAfterFilter);   //Get envelope by squaring the input
    envelope = (envelope > threshold) ? envelope : 0;    // The data set below the base value is set to 0, indicating that it is in a relaxed state

    /* if threshold=0,explain the status it is in the calibration process,the code bollow not run.
      if get EMG singal,number++ and print
    */
  if (threshold > 0)
  {
    if (getEMGCount(envelope))
    {
      EMG_num++;
      Serial.print("EMG_num: ");
      Serial.println(EMG_num);
    }
  }
  else {
    Serial.println(envelope);
  }
  delayMicroseconds(500);
}

/*
   if get EMG signal,return 1;
*/
int getEMGCount(int gforce_envelope)
{
  static long integralData = 0;
  static long integralDataEve = 0;
  static bool remainFlag = false;
  static unsigned long timeMillis = 0;
  static unsigned long timeBeginzero = 0;
  static long fistNum = 0;
  static int  TimeStandard = 200;
  /*
    The integral is processed to continuously add the signal value
    and compare the integral value of the previous sampling to determine whether the signal is continuous
   */
  integralDataEve = integralData;
  integralData += gforce_envelope;
  /*
    If the integral is constant, and it doesn't equal 0, then the time is recorded;
    If the value of the integral starts to change again, the remainflag is true, and the time record will be re-entered next time
  */
  if ((integralDataEve == integralData) && (integralDataEve != 0))
  {
    timeMillis = millis();
    if (remainFlag)
    {
      timeBeginzero = timeMillis;
      remainFlag = false;
      return 0;
    }
    /* If the integral value exceeds 200 ms, the integral value is clear 0,return that get EMG signal */
    if ((timeMillis - timeBeginzero) > TimeStandard)
    {
      integralDataEve = integralData = 0;
      return 1;
    }
    return 0;
  }
  else {
    remainFlag = true;
    return 0;
   }
  // delay(1000);
}
