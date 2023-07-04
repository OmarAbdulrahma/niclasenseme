#include <MovingAveragePlus.h>
#include <ArduinoBLE.h>       //A general thanks to Klaus_K from the Arduino forum for helping with BLE data transmission
#include "Arduino_BHY2.h"
#include "Nicla_System.h"
MovingAveragePlus<unsigned long> stepLengthAvg(3);  //Makes MovingAvg object with 3 points
MovingAveragePlus<float> peakValueAvg(3);

//----------------------------------------------------------------------------------------------------------------------
// User-friendly variable stuff
//----------------------------------------------------------------------------------------------------------------------

#define RECORDING_LENGTH_SECONDS 15           //Length of each trial (seconds)
#define VIBRATION_START_PERCENTAGE_MOTOR1 0   //Percentage of gait cycle when vibration activates (20 = motor starts 20% after HS) motor1= upper motor
#define VIBRATION_START_PERCENTAGE_MOTOR2 30  //MOTOR2=lower motor
#define VIBRATION_END_PERCENTAGE 60           //Percentage of gait cycle when vibration deactivates (60 = motor stops 60% after HS)
#define STEP_LENGTH_THRESHOLD 1.5             //Percentage at which step length won't be added to stepLengthAvg (1.5 = 150%)
int SAMPLES_TO_IGNORE_AFTER_HS = 5;           //Prevents early detection of HS on a spike
float MS_THRESHOLD_PCT_OF_PEAK = 0.5;         //Percentage of moving avg peak value for MS detection threshold
float INITIAL_MS_THRESHOLD = 100;             //Fixed-value threshold for initial two steps

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------

#define BLE_UUID_SENSOR_SERVICE "19120000-1713-4AE2-5A50-946B02A2FFAF"
#define BLE_UUID_SENSOR "19120001-1713-4AE2-5A50-946B02A2FFAF"
#define BLE_UUID_SENSOR_CONTROL "19120002-1713-4AE2-5A50-946B02A2FFAF"
#define BLE_UUID_CHARACTERISTIC_USER_DESCRIPTION "2901"
#define BLE_UUID_STIM_CONTROL "19120003-1713-4AE2-5A50-946B02A2FFAF"

//----------------------------------------------------------------------------------------------------------------------
// BLE Setup
//----------------------------------------------------------------------------------------------------------------------

// typedef struct _attribute_((packed)) {
//   uint32_t position;
//   uint32_t time;
// } sensor_data_t;
typedef struct {
  uint32_t position;
  uint32_t time;
} __attribute__((packed)) sensor_data_t;

typedef union {
  sensor_data_t values;
  uint8_t bytes[sizeof(sensor_data_t)];
} sensor_data_ut;

sensor_data_ut sensorData;

bool sensorDataUpdated = false;
//----- DEVICE NAME -----//
#define BLE_DEVICE_NAME "GaitGenie"
#define BLE_LOCAL_NAME "GaitGenie"

BLEService genieSensorService(BLE_UUID_SENSOR_SERVICE);
BLECharacteristic sensorCharacteristic(BLE_UUID_SENSOR, BLERead | BLENotify, sizeof(sensor_data_ut));
BLEByteCharacteristic controlCharacteristic(BLE_UUID_SENSOR_CONTROL, BLERead | BLEWrite);
BLECharacteristic controlSTIMCharacteristic(BLE_UUID_STIM_CONTROL, BLERead | BLEWrite | BLENotify,20);
//BLEDescriptor controlDescriptor(BLE_UUID_CHARACTERISTIC_USER_DESCRIPTION, "CMD:01-RESET,02-RECORD,03-REPLAY");

//----------------------------------------------------------------------------------------------------------------------
// APP & I/O
//----------------------------------------------------------------------------------------------------------------------

// Reversing Byte order allows uint32_t data to be converted in Excel with HEX2DEC function OR MATLAB code
#define REVERSE_BYTE_ORDER

#define DATA_BUFFER_SIZE (RECORDING_LENGTH_SECONDS * 100)  // 20 sec X 100 Hz    Changes buffer size

enum BUFER_STATE_TYPE { BUFFER_STATE_READY,
                        BUFFER_STATE_RECORD,
                        BUFFER_STATE_REPLAY,
                        BUFFER_STATE_FULL
};

// typedef struct _attribute_((packed)) {
//   sensor_data_t values[DATA_BUFFER_SIZE];
//   uint32_t index;
//   uint32_t state = BUFFER_STATE_READY;
// } sensor_buffer_t;
typedef struct {
  sensor_data_t values[DATA_BUFFER_SIZE];
  uint32_t index;
  uint32_t state;
} __attribute__((packed)) sensor_buffer_t;

sensor_buffer_t dataBuffer;
bool sensorActive = false;

SensorXYZ gyroscope(SENSOR_ID_GYRO);
SensorXYZ accelerometer(SENSOR_ID_ACC);

//----------------------------------------------------------------------------------------------------------------------
// Setup
//----------------------------------------------------------------------------------------------------------------------

void setup() {
  // Initialize and Setup IMU
 Serial.begin(115200);

  Serial.println("Start");

  nicla::begin();
  nicla::leds.begin();
  nicla::leds.setColor(green);

  //Sensors initialization
  BHY2.begin(NICLA_STANDALONE);
  
  gyroscope.begin();
  accelerometer.begin();
  //quaternion.begin();

 
  if (!setupBleMode()) {
    //Serial.println( "Failed to initialize BLE!" );
    while (1) {
      nicla::leds.setColor(red);
      delay(400);
      nicla::leds.setColor(blue);;
      delay(400);
    }
  } else {
    //Serial.println( "BLE initialized. Waiting for clients to connect." );
    nicla::leds.setColor(green);
    delay(500);
    nicla::leds.setColor(blue);
    delay(500);
  }
}

//----------------------------------------------------------------------------------------------------------------------
// Variable Declarations
//----------------------------------------------------------------------------------------------------------------------

//For HS Detection HS=heel-strike
float preVal = 0;
float curVal = 0;

//Gait phase markers
bool MSDetected = false;
bool ZeroCrossP2N = false;  //Crossing from Positive sign to Negative
bool HSDetected = false;
bool firstGyro = true;
unsigned long previousMillis = 0;

//For timing considerations
bool firstHS = true;
bool secondHS = true;
unsigned long firstMillis;
unsigned long newMillis;
unsigned long oldMillis;

//Vibration Task
bool readyToVibrate = false;
bool vibrationPhase = false;
unsigned long currVibeMillis;
unsigned long prevVibeMillis;

// HS detection
unsigned long currDetectMillis;
unsigned long prevDetectMillis;
int sampleCounter = 0;

//MS detection MS=mid-swing
float MS_THRESHOLD = INITIAL_MS_THRESHOLD;  //Minimum value that only above it a mid-swing might be detected. It prevents false positive
float peakValue;
bool checkForPeak = true;
bool setMSThreshold = false;


//For moving average
static int AVG_ARRAY_LENGTH = 3;
unsigned long stepLengthAverage = 0;
//Stim FES command
String stim = "";
//----------------------------------------------------------------------------------------------------------------------
//  Loop
//----------------------------------------------------------------------------------------------------------------------

void loop() {
  BHY2.update();
  bleTask();
  sensorTask();
  //sendDataTask(RECORDING_LENGTH_SECONDS);  //UPDATE13/12/2021:This task is relevant only when we want to resend the data AKA REPLAY 03.
  vibrationTask();
}

//----------------------------------------------------------------------------------------------------------------------
//  BLE Task
//----------------------------------------------------------------------------------------------------------------------

void bleTask() {
  const uint32_t BLE_UPDATE_INTERVAL = 10;
  static uint32_t previousMillis = 0;

  uint32_t currentMillis = millis();
  if (currentMillis - previousMillis >= BLE_UPDATE_INTERVAL) {
    previousMillis = currentMillis;
    BLE.poll();
  }

  if (sensorDataUpdated) {
    sensorDataUpdated = false;
    sensorCharacteristic.writeValue(sensorData.bytes, sizeof sensorData.bytes);
  }
}

enum CMD_TYPE { CMD_RESET_BUFFER = 1,
                CMD_RECORD,
                CMD_REPLAY
                //                CMD_DISCOPARTY #Respect Omri, NB
};


// void controlCharacteristicWrittenHandler(BLEDevice central, BLECharacteristic bleCharacteristic) {
//   //Serial.print( "BLE characteristic written. UUID: " );
//   //Serial.println( bleCharacteristic.uuid() );

//   uint8_t cmd;
//   if (bleCharacteristic.uuid() == (const char*)BLE_UUID_SENSOR_CONTROL) {
//     bleCharacteristic.readValue(cmd);
//     ////Serial.print( " CMD: " );
//     ////Serial.println( cmd, HEX );
//     switch (cmd) {
//       case CMD_RESET_BUFFER:
//         dataBuffer.index = 0;
//         dataBuffer.state = BUFFER_STATE_READY;
//         //digitalWrite(BUFFER_LED_PIN, LOW);
//         ////Serial.println( " Reset" );
//         break;
//       case CMD_RECORD:
//         dataBuffer.index = 0;
//         dataBuffer.state = BUFFER_STATE_RECORD;
//         //digitalWrite(BUFFER_LED_PIN, LOW);
//         sensorActive = true;
//         ////Serial.println( " Record" );
//         firstHS = true;
//         secondHS = true;
//         break;
//       case CMD_REPLAY:
//         dataBuffer.index = 0;
//         dataBuffer.state = BUFFER_STATE_REPLAY;
//         //digitalWrite(BUFFER_LED_PIN, LOW);
//         sensorActive = false;
//         ////Serial.println( " Replay" );
//         break;

//       default:
//         //Serial.println( " unknown" );
//         break;
//     }
//   }
// }
//----------------------------------------------------------------------------------------------------------------------
//  Sensor Task
//----------------------------------------------------------------------------------------------------------------------

void sensorTask() {
  const uint32_t SENSOR_SAMPLING_INTERVAL = 10;  //Update 13.09.2022: Affect the Sample rate. 10 gives 95 in recording 03. 15 gives 60Hz. 7 gives noisy/spikey 105Hz.
                                                 //For some reason, it does not record well on my phone for some parameters.
  static uint32_t previousMillis = 0;


  // if (!sensorActive) {  //This statement is true if we are in mode 03, meaning "replay" data and not measuring.
  //   return;
  // }

  uint32_t currentMillis = millis();
  if (currentMillis - previousMillis >= SENSOR_SAMPLING_INTERVAL) {
    previousMillis = currentMillis;

    float gyroX, gyroY, gyroZ;

   // if (IMU.gyroAvailable()) {
      //IMU.readGyro(gyroX, gyroY, gyroZ);
      gyroX = gyroscope.x();
      gyroY = gyroscope.y();
      gyroZ = gyroscope.z();
      HeelStrikeDetect(gyroY);
      firstGyro = false;
     // int32_t gyroYint = (int32_t)round(gyroY * 1000000);  //I chose a factor of 1 million to get a good compromise between digits on either side of the dot. You can choose another factor depending on your value range.
      //sensorData.values.position = gyroYint;
      //sensorData.values.time = currentMillis;
   // }

// #ifdef REVERSE_BYTE_ORDER
//     sensorData.values.position = __REV(sensorData.values.position);
//     sensorData.values.time = __REV(sensorData.values.time);
// #endif

//     //        sensorDataUpdated = true;  //Used to be line 356. uncomment to record when sending 02!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//     if (dataBuffer.state == BUFFER_STATE_RECORD)  //See if the buffer is ready to record, meaning 02 was sent.
//     {
//       dataBuffer.values[dataBuffer.index] = sensorData.values;
//       dataBuffer.index = (dataBuffer.index + 1) % DATA_BUFFER_SIZE;
//       if (dataBuffer.index == 0) {
//         dataBuffer.state = BUFFER_STATE_FULL;
//         //digitalWrite(BUFFER_LED_PIN, HIGH);
//         sensorActive = false;
//         //Serial.println( "Recording done" );
//       }
//     }
  }
}

//----------------------------------------------------------------------------------------------------------------------
//  Send Data Task
//----------------------------------------------------------------------------------------------------------------------

// int32_t sendDataTask(uint32_t interval)  //This task is only for when we want to resend the data AKA REPLAY
// {
//   const uint32_t SEND_DELAY_MS = 5000;  //Wait 5 sec from when REPLAY was asked to when it is republished
//   static uint32_t previousMillis = 0;
//   static bool startUpDelay = true;

//   if (dataBuffer.state != BUFFER_STATE_REPLAY) {  //See if the buffer is ready to replay data, meaning 03 was sent. If not then return to loop.
//     return -1;
//   }

//   if (dataBuffer.index == 0) {
//     if (startUpDelay) {
//       startUpDelay = false;
//       previousMillis = millis();
//       // Serial.println( "Get ready for send" );
//       return -1;
//     }
//     uint32_t currentMillis = millis();
//     if (currentMillis - previousMillis < SEND_DELAY_MS) {
//       return -1;
//     } else {
//       startUpDelay = true;
//       // Serial.println( "Sending data" );
//     }
//   }

//   uint32_t currentMillis = millis();
//   if (currentMillis - previousMillis >= interval) {
//     previousMillis = currentMillis;

//     sensorData.values = dataBuffer.values[dataBuffer.index];
//     sensorDataUpdated = true;
//     dataBuffer.index = (dataBuffer.index + 1) % DATA_BUFFER_SIZE;
//     if (dataBuffer.index == 0) {
//       dataBuffer.state = BUFFER_STATE_READY;
//       //digitalWrite(BUFFER_LED_PIN, HIGH);
//       // Serial.println( "Sending done" );
//     }
//     return dataBuffer.index;
//   }
//   return -1;
// }

//----------------------------------------------------------------------------------------------------------------------
// Vibration Task
//----------------------------------------------------------------------------------------------------------------------

void vibrationTask() {
  if (readyToVibrate) {
    currVibeMillis = millis();

    if (currVibeMillis - prevVibeMillis >= stepLengthAvg.get() * VIBRATION_START_PERCENTAGE_MOTOR1 * 0.01) {  //Delay before vibration starts

      //digitalWrite(MOTOR_LED, HIGH);  //LED on
      stim="FES_ON";
      controlSTIMCharacteristic.writeValue(stim.c_str(), stim.length());
      vibrationPhase = true;
      readyToVibrate = false;
      Serial.println("Fireing ....FES_ON");
    }
  }

  if (vibrationPhase) {
    currVibeMillis = millis();

    if (currVibeMillis - prevVibeMillis >= stepLengthAvg.get() * (VIBRATION_END_PERCENTAGE - VIBRATION_START_PERCENTAGE_MOTOR1) * 0.01) {  //Duration of vibration until stop
      stim="FES_OFF";
      controlSTIMCharacteristic.writeValue(stim.c_str(), stim.length());
      //digitalWrite(MOTOR_LED, LOW);  //LED off
      vibrationPhase = false;
      prevVibeMillis = currVibeMillis;
      Serial.println("Fireing ....FES_OFF");
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
// Helper Methods / Handlers / Functions
//----------------------------------------------------------------------------------------------------------------------

void HeelStrikeDetect(float val) {

  //Detection delay to ensure no false positives after heel-strike. Depending on the sample rate, this value should be high for a high sample rate.
  if (sampleCounter <= SAMPLES_TO_IGNORE_AFTER_HS) {
    sampleCounter++;
    return;
  }

  if (firstGyro) {
    curVal = val;
  } else {
    preVal = curVal;
    curVal = val;

    if (curVal >= MS_THRESHOLD) {  //Checks mid-swing occurring
      //Serial.print("MS_THRESHOLD: ");
      //Serial.println(MS_THRESHOLD);
      if (preVal >= curVal && checkForPeak == true) {  //Finds mid-swing value and adds to avg
        peakValue = preVal;
        //Serial.print("--------PEAK VALUE: ");
        //Serial.println(peakValue);
        peakValueAvg.push(peakValue);
        //Serial.print("--------AVG VALUE: ");
        //Serial.println(peakValueAvg.get());
        checkForPeak = false;
      }
      if (setMSThreshold == true) {  //Sets threshold to moving avg after second step
        //Serial.println("SET THRESH TO AVG");
        MS_THRESHOLD = MS_THRESHOLD_PCT_OF_PEAK * peakValueAvg.get();
      }

      MSDetected = true;  //For creating a mid-swing event-dependent feedback, insert a statement here to trigger the desired feedback.
      ZeroCrossP2N = false;
    }

    if (MSDetected == true && preVal >= 0 && curVal <= 0) {  //Checks zero cross
      ZeroCrossP2N = true;
      MSDetected = false;
    }

    if (ZeroCrossP2N == true) {
      if (preVal < curVal) {  //After mid-swing and slope goes from positive to negative (ZeroCrossP2N true) we know a heel-strike is coming. When the preVal is smaller than curVal, heel-strike has just happened on the prev sample.
        MSDetected = false;
        ZeroCrossP2N = false;
        sampleCounter = 0;

        if (firstHS) {
          firstMillis = millis();
          firstHS = false;
        } else if (secondHS && firstMillis - newMillis > 350) {  //Ensures that the first point added to the average is not too short
          newMillis = millis();
          stepLengthAvg.push(newMillis - firstMillis);
          stepLengthAverage = newMillis - firstMillis;  //Not average yet, updates after the third step
          //Serial.print("Step Length (ms): ");
          //Serial.println(newMillis - firstMillis);
          secondHS = false;
        } else {
          oldMillis = newMillis;  //Sets old to the previous new time
          newMillis = millis();

          if (newMillis - oldMillis <= STEP_LENGTH_THRESHOLD * stepLengthAverage) {  //Step must be under 150% of prev step to updated step length avg.
            //Serial.print("Step Length (ms): ");
            //Serial.println(newMillis - oldMillis);
            stepLengthAvg.push(newMillis - oldMillis);
            stepLengthAverage = stepLengthAvg.get();
          } else {
          }
          setMSThreshold = true;  //Sets the threshold to moving MS avg after the second step
        }

        readyToVibrate = true;
        checkForPeak = true;
        prevVibeMillis = millis();
      }
    }
  }
}



//BLE helpers
void blePeripheralConnectHandler(BLEDevice central) {
  nicla::leds.setColor(blue);
}


void blePeripheralDisconnectHandler(BLEDevice central) {
 nicla::leds.setColor(green);
}


bool setupBleMode() {
  if (!BLE.begin()) {
    nicla::leds.setColor(red);
    delay(500);
    return false;
  }

  // Set advertised local name and service UUID
  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setLocalName(BLE_LOCAL_NAME);
  BLE.setAdvertisedService(genieSensorService);

  // BLE add characteristics
  //genieSensorService.addCharacteristic(sensorCharacteristic);
  //genieSensorService.addCharacteristic(controlCharacteristic);
  genieSensorService.addCharacteristic(controlSTIMCharacteristic);
  // BLE add descriptors
  //controlCharacteristic.addDescriptor(controlDescriptor);

  // Add service
  BLE.addService(genieSensorService);

  // Set the initial value for the characteristic
  //sensorCharacteristic.writeValue(sensorData.bytes, sizeof sensorData.bytes);
  //controlCharacteristic.writeValue(0);

  // Set BLE event handlers
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Set service and characteristic-specific event handlers
  //controlCharacteristic.setEventHandler(BLEWritten, controlCharacteristicWrittenHandler);

  // Start advertising
  BLE.advertise();

  return true;
}