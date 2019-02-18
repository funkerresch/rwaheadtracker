    #include <Wire.h>
    #include "SparkFun_BNO080_Arduino_Library.h"
    #include "utility/imumaths.h"
    #include <math.h>
    #include <BLEDevice.h>
    #include <BLEUtils.h>
    #include <BLEServer.h>
    #include <BLE2902.h>
    #include "soc/soc.h"
    #include "soc/rtc_cntl_reg.h"
    #include "sdkconfig.h"

//************************************************************************/
// Sparkfun BNO080 Orientation Sensor
//************************************************************************/
      
    BNO080 bno080;//test
    #define BUF_LEN 20
    byte byteBuffer[BUF_LEN];
    char charBuffer[BUF_LEN] = {0x00};
    String deviceName = "rwaht00";
    
//************************************************************************/
// Bluethooth Setup
//************************************************************************/

// Deactivate brown out detection
#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);
//=============================================

BLECharacteristic *pCharacteristicTracking;
bool deviceConnected = false;
uint8_t value = 0;

#define SERVICE_UUID          "713D0000-503E-4C75-BA94-3148F18D941E"
#define CHARACTERISTIC_UUID   "713D0002-503E-4C75-BA94-3148F18D941E"

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
  // Here I get the commands from the App (client)
    void onWrite(BLECharacteristic *pCharacteristicTracking) {
      std::string value = pCharacteristicTracking->getValue();

      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

     void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

const int LED_PIN = 4;  // Show BLE connection status
const int BAT_PIN = A13; // Messure half the battery voltage
float batVoltage = 0;  
 
void setup(void) {
      // Deactivate brown out detection
      //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
      
  Serial.begin(115200);
  Serial.println("BNO080 ESP32 Headtracker");


  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  // Activate IMU functionalities
  bno080.begin();

  //bno080.calibrateAll();
  bno080.enableRotationVector(20);       
  bno080.enableLinearAccelerometer(20);    
  //bno080.enableStepCounter(500);   // Kommt anscheinend mit LinearAccelerometer in Konflikt - getrennt testen         
  
  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Linear Accelerometer enabled"));
  Serial.println(F("Step counter enabled"));
  Serial.println(F("Output in form yaw, pitch, lin_accel_z, step_count and in new line accuracy"));
      
  delay(1000);     

// Bluetooth Setup
  Serial.println("1- Download and install an BLE scanner app in your phone");
  Serial.println("2- Scan for BLE devices in the app");
  Serial.println("3- Connect to BNO080 ESP32 Headtracker");
  Serial.println("4- Go to CUSTOM CHARACTERISTIC in CUSTOM SERVICE and write something");
  Serial.println("5- See the magic =)");
  
  batVoltage = 0.002 * analogRead(BAT_PIN);
  Serial.print("Battery: ");
  Serial.print(batVoltage);
  Serial.println(" V");
  // TODO: Buzzer peep tone while low power
  

  BLEDevice::init(deviceName.c_str());
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pCharacteristicTracking = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ   |
                                         BLECharacteristic::PROPERTY_WRITE  |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristicTracking->addDescriptor(new BLE2902());

  pCharacteristicTracking->setCallbacks(new MyCharacteristicCallbacks());

  pCharacteristicTracking->setValue(deviceName.c_str());
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

}
    
bool disconnectedMessage = false;
enum dataFormat {
  stringFormat, // current format
  byteFormat,
  oscFormat,
  customFormat 
  
  };

    
    dataFormat currentDataFormat = stringFormat;
    /* To Do */
    //dataFormat currentDataFormat = byteFormat;
    // dataFormat currentDataFormat = oscFormat;
    // dataFormat currentDataFormat = customFormat;
    
void loop(void) {
      
      if (deviceConnected == true) {
        digitalWrite(LED_PIN, HIGH);
        disconnectedMessage = false;

        // Getting sensor data
        
        switch (currentDataFormat) {
          case byteFormat:
            //Serial.println("sending byte format");
          
          break;

          case stringFormat:
            //Serial.println("sending string format");
            sendingInStringFormat();
          break;

          case oscFormat:
            //Serial.println("sending osc format");
      
          break;

           case customFormat:
            //Serial.println("sending custom format");
      
          break;
          
          default:
            Serial.println("error: undefined data transfer format");
          break;
        }
       
      
       

  } else {
    if (disconnectedMessage == false) {
     Serial.println("not connected\n...\n");
     disconnectedMessage = true;
     delay(1000);
    } else {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(500);
    }
  }
      
      delay(10); // Maybee reduce delay time?
}

    /*
        roll  = Mathf.Atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
        pitch = Mathf.Atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z);
        yaw   =  Mathf.Asin(2*x*y + 2*z*w);
    */


    void  sendingInStringFormat() {
      // TODO: Separate getting values from sending values
      
      if (bno080.dataAvailable() == true) {
        float quatI = bno080.getQuatI();
        float quatJ = bno080.getQuatJ();
        float quatK = bno080.getQuatK();
        float quatReal = bno080.getQuatReal();

        imu::Quaternion quat = imu::Quaternion(quatReal, quatI, quatJ, quatK);
        quat.normalize();
        imu::Vector<3> q_to_euler = quat.toEuler();
       
      
      int yaw_degrees, pitch_degrees;
      float yaw_degrees_f, pitch_degrees_f, lin_accel_z_f;
      String yaw_degrees_str, pitch_degrees_str, lin_accel_z_str, step_str, dataStr;
      float yaw = q_to_euler.x();
      
      yaw_degrees_f = yaw * -180.0 / M_PI; // conversion to degrees
      if( yaw_degrees_f < 0 ) 
        yaw_degrees_f += 360.0; // convert negative to positive angles
      yaw_degrees = (int)(round(yaw_degrees_f));
      yaw_degrees_str = String(yaw_degrees);
      
      float pitch = q_to_euler.z();
      pitch_degrees_f = pitch * -180.0/ M_PI;
   //   if( pitch_degrees < 0 ) 
   //     pitch_degrees += 180.0; // convert negative to positive angles
      pitch_degrees = (int)(round(pitch_degrees_f));
      pitch_degrees_str = String(pitch_degrees);

      lin_accel_z_f = 0.0;
      lin_accel_z_f = bno080.getLinAccelZ(); // float
      lin_accel_z_str = String(lin_accel_z_f);

      unsigned int steps = 0;
     // steps = bno080.getStepCount();
      step_str = String(steps);
      // Set coordinates
      
      
      // Data package
      dataStr = yaw_degrees_str + " " + pitch_degrees_str + " " + lin_accel_z_str + " " + step_str;
      // Send data
      pCharacteristicTracking->setValue(dataStr.c_str());
      // Say here is new data
      pCharacteristicTracking->notify();
      // Print data
      Serial.println(dataStr);
    
      
       //Look for reports from the IMU
 
   
    float quatRadianAccuracy = bno080.getQuatRadianAccuracy();
    Serial.print(F("QuatRadianAccuracy: "));
    Serial.print(quatRadianAccuracy, 2);
    uint64_t chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    Serial.printf("\nESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
    Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.
  
      }
       
    }

      

    /**************************************************************************/
    /*
        Display sensor calibration status
    */
    /**************************************************************************/
    /*
     * 
     */

     /*
     void displayCalStatus(void)
    {
      // Get the four calibration values (0..3) 
      // Any sensor data reporting 0 should be ignored, 
      // 3 means 'fully calibrated" 
      uint8_t system, gyro, accel, mag;
      system = gyro = accel = mag = 0;

      bno.getCalibration(&system, &gyro, &accel, &mag);
     
      // The data should be ignored until the system calibration is > 0 
      Serial.print("\t");
      if (!system)
      {
        Serial.print("! ");
      }
     
      // Display the individual values 
      Serial.print("Sys:");
      Serial.print(system, DEC);
      Serial.print(" G:");
      Serial.print(gyro, DEC);
      Serial.print(" A:");
      Serial.print(accel, DEC);
      Serial.print(" M:");
      Serial.println(mag, DEC);
    }
*/
