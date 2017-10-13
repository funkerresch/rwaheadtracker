#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (25)
#define MIN_CONN_INTERVAL          0x0006 
#define MAX_CONN_INTERVAL          0x0018 
#define SLAVE_LATENCY              0x0000 
#define CONN_SUPERVISION_TIMEOUT   0x03E8 // 10s.
#define BLE_PERIPHERAL_APPEARANCE  BLE_APPEARANCE_UNKNOWN
#define BLE_DEVICE_NAME            "RWA_HEADTRACKER"
#define CHARACTERISTIC1_MAX_LEN    20
#define CHARACTERISTIC2_MAX_LEN    20
#define TXRX_BUF_LEN               20

int yaw_degrees = 0;
int lastYaw_degrees = -1;
int pitch_degrees = 0;
int lastPitch_degrees = -1;

imu::Vector<3> accel;
imu::Vector<3> gyro;

bool sendRawGyro = 0;
bool sendRawAccel = 1;
bool sendSteps = 1;
bool sendStepSerial = false;

bool withSerial = 1;
bool withBLE = 1;
bool withWIFI = 0;

bool stepBlocked = true;
bool stepPattern = false;
long startTimeForStepPattern = 0;
long stepTimeStamp = 0;
long currentTimeStamp = 0;
long measuredTimeSinceLastStep = 0;
long timeThreshSinceLastStep = 400;
long stepCounter = 0;
long timeThreshForStepPattern = 100;
float threshholdForStep = 10.2;

static uint8_t adv_data[] = 
{
    0x02,
    BLE_GAP_AD_TYPE_FLAGS,
    BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,
  
    0x08,
    BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
     'r','w','a','h','t','0','4',
  
    0x11,
    BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE,
    0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,0xba,0x75,0x4c,0x3e,0x50,0x00,0x00,0x3d,0x71
};

static uint8_t service1_uuid[16]    = { 0x71,0x3d,0x00,0x00,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_rx_uuid[16] = { 0x71,0x3d,0x00,0x02,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };


// GAP and GATT characteristics value
static uint8_t  appearance[2] = 
{ 
    LOW_BYTE(BLE_PERIPHERAL_APPEARANCE), 
    HIGH_BYTE(BLE_PERIPHERAL_APPEARANCE) 
};

static uint8_t  change[4] = {
    0x00, 0x00, 0xFF, 0xFF
};

static uint8_t  conn_param[8] = 
{
    LOW_BYTE(MIN_CONN_INTERVAL), HIGH_BYTE(MIN_CONN_INTERVAL), 
    LOW_BYTE(MAX_CONN_INTERVAL), HIGH_BYTE(MAX_CONN_INTERVAL), 
    LOW_BYTE(SLAVE_LATENCY), HIGH_BYTE(SLAVE_LATENCY), 
    LOW_BYTE(CONN_SUPERVISION_TIMEOUT), HIGH_BYTE(CONN_SUPERVISION_TIMEOUT)
};

static advParams_t adv_params = 
{
    .adv_int_min   = 0x001E,
    .adv_int_max   = 0x001E,
    .adv_type      = BLE_GAP_ADV_TYPE_ADV_IND,
    .dir_addr_type = BLE_GAP_ADDR_TYPE_PUBLIC,
    .dir_addr      = {0,0,0,0,0,0},
    .channel_map   = BLE_GAP_ADV_CHANNEL_MAP_ALL,
    .filter_policy = BLE_GAP_ADV_FP_ANY
};


static uint16_t character2_handle = 0x0000;
static uint8_t characteristic2_data[CHARACTERISTIC2_MAX_LEN] = { 0x00 };
static btstack_timer_source_t characteristic2;

char rx_buf[TXRX_BUF_LEN];
char serial_buf[TXRX_BUF_LEN];
static uint8_t rx_state = 0;
static uint8_t sendStepState = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(1111);
int offsetForImu = 0;

#if defined(ARDUINO) 
SYSTEM_MODE(MANUAL); 
#endif

/* WLAN RELATED */

UDP Udp;
char ssid[] = "OSPW";
char password[] = "12OSPW3456";
unsigned int localPort = 3002;      
IPAddress timeServer(192,168,43,228);
byte packetBuffer[TXRX_BUF_LEN]; 

void sendNTPpacket(IPAddress& address, int ID, float value);
void printWifiStatus();

void deviceConnectedCallback(BLEStatus_t status, uint16_t handle) {
  switch (status) {
    case BLE_STATUS_OK:
      Serial.println("Device connected!");
      break;
    default: break;
  }
}

void deviceDisconnectedCallback(uint16_t handle){
  Serial.println("Disconnected.");
}

int gattWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {
  Serial.print("Write value handler: ");
  Serial.println(value_handle, HEX);

  /*if (character1_handle == value_handle) {
    memcpy(characteristic1_data, buffer, min(size,CHARACTERISTIC1_MAX_LEN));
    Serial.print("Characteristic1 write value: ");
    for (uint8_t index = 0; index < min(size,CHARACTERISTIC1_MAX_LEN); index++) {
      Serial.print(characteristic1_data[index], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
  }*/
  return 0;
}

void setup() 
{      
    Serial.begin(115200);
    
    if(!bno.begin())
    {
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    } 
    
    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;   
    bno.getSensor(&sensor);
        
    int eeAddress = 0;
    long bnoID;
    EEPROM.get(eeAddress, bnoID);
    
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(250);
    }
    
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);
        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);
        Serial.println("\n\nCalibration data loaded into BNO055");
    }

    bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
    bno.setExtCrystalUse(true);
    
    if(withWIFI)
    {
        // attempt to connect to Wifi network:
        Serial.print("Attempting to connect to Network named: ");
        Serial.println(ssid); 
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        
        WiFi.on();
        WiFi.setCredentials(ssid, password);
        WiFi.connect();
        
        while (WiFi.connecting()) 
        {
            Serial.print(".");
            delay(300);
        }
        
        Serial.println("\nYou're connected to the network");
        Serial.println("Waiting for an ip address");
        
        IPAddress localIP = WiFi.localIP();
        while (localIP[0] == 0) 
        {
            localIP = WiFi.localIP();
            Serial.println("waiting for an IP address");
            delay(1000);
        }
      
        Serial.println("\nIP Address obtained");
        
        // you're connected now, so print out the status  
        printWifiStatus();   
        Serial.println("\nStarting connection to server...");
        Udp.begin(localPort);
    }

    if(withBLE)
    {      
        Serial.println("RWA_HEADTRACKER");
        ble.init();
      
        // Register BLE callback functions
        ble.onConnectedCallback(deviceConnectedCallback);
        ble.onDisconnectedCallback(deviceDisconnectedCallback);
        ble.onDataWriteCallback(gattWriteCallback);      
      
        // Add GAP service and characteristics
        ble.addService(BLE_UUID_GAP);
        ble.addCharacteristic(BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME, ATT_PROPERTY_READ|ATT_PROPERTY_WRITE, (uint8_t*)BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME));
        ble.addCharacteristic(BLE_UUID_GAP_CHARACTERISTIC_APPEARANCE, ATT_PROPERTY_READ, appearance, sizeof(appearance));
        ble.addCharacteristic(BLE_UUID_GAP_CHARACTERISTIC_PPCP, ATT_PROPERTY_READ, conn_param, sizeof(conn_param));
      
        // Add GATT service and characteristics
        ble.addService(BLE_UUID_GATT);
        ble.addCharacteristic(BLE_UUID_GATT_CHARACTERISTIC_SERVICE_CHANGED, ATT_PROPERTY_INDICATE, change, sizeof(change));
      
        // Add user defined service and characteristics
        ble.addService(service1_uuid);
        //character1_handle = ble.addCharacteristicDynamic(service1_tx_uuid, ATT_PROPERTY_NOTIFY|ATT_PROPERTY_WRITE|ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, characteristic1_data, CHARACTERISTIC1_MAX_LEN);
        character2_handle = ble.addCharacteristicDynamic(service1_rx_uuid, ATT_PROPERTY_NOTIFY, characteristic2_data, CHARACTERISTIC2_MAX_LEN);
        
        // Set BLE advertising parameters
        ble.setAdvertisementParams(&adv_params);
      
        // // Set BLE advertising data
        ble.setAdvertisementData(sizeof(adv_data), adv_data);
      
        // BLE peripheral starts advertising now.
        ble.startAdvertising();
        Serial.println("BLE start advertising.");
      
        // set one-shot timer
        characteristic2.process = &characteristic2_notify;
        ble.setTimer(&characteristic2, 500);//100ms
        ble.addTimer(&characteristic2);  
    }
}

static void  characteristic2_notify(btstack_timer_source_t *ts) 
{       
    if (rx_state) 
    {      
        sprintf(rx_buf, "%d %d %.2f", yaw_degrees, pitch_degrees, accel[2]);
        ble.sendNotify(character2_handle, (uint8_t*)rx_buf, strlen(rx_buf));       
        memset(rx_buf, 0x00, strlen(rx_buf));
        rx_state = 0;   
    }
    
    ble.setTimer(ts, BNO055_SAMPLERATE_DELAY_MS);
    ble.addTimer(ts);
}

bool yawSignificantlyChanged()
{
   
  
}

void loop() 
{
    currentTimeStamp = millis();
    accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   
   
    imu::Quaternion quat = bno.getQuat();
    quat.normalize();
    imu::Vector<3> q_to_euler = quat.toEuler();
     
    double yaw = q_to_euler.x();
    yaw_degrees = yaw * -180.0 / M_PI; // conversion to degrees
    if( yaw_degrees < 0 ) 
        yaw_degrees += 360.0; // convert negative to positive angles

    double pitch = q_to_euler.y();
    pitch_degrees = pitch * -180 / M_PI;
    if( pitch_degrees < 0 ) 
        pitch_degrees += 360.0; // convert negative to positive angles   
    
    if(withWIFI)
          sendNTPpacket(timeServer, 1, yaw_degrees); 
    }
    if(withBLE)
          rx_state = 1;         
    
    if(withSerial)
    {                 
         sprintf(serial_buf, "%d %d %.2f", yaw_degrees, pitch_degrees, accel[2]);
         Serial.println(serial_buf);         
    }
    
    lastYaw_degrees = yaw_degrees;
    lastPitch_degrees = pitch_degrees;
    delay(BNO055_SAMPLERATE_DELAY_MS);
 }

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address, int ID, float value) 
{
    memset(packetBuffer, 0, TXRX_BUF_LEN);
    longToByte(packetBuffer, ID);
    floatToByte(&packetBuffer[4], value);
    Udp.beginPacket(address, 3002); 
    Udp.write(packetBuffer, TXRX_BUF_LEN);
    Udp.endPacket();
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void longToByte(byte* arr, long value)
{
    arr[0] = (byte) value;
    arr[1] = (byte) value >> 8;
    arr[2] = (byte) value>> 16;
    arr[3] = (byte) value >> 24;
}

long byteToLong(byte* arr)
{
    long value = (unsigned long)(arr[4] << 24) | (arr[3] << 16) | (arr[2] << 8) | arr[1];
    return value;
}

float byteToFloat(byte* arr)
{
     int i = arr[0] | (arr[1] << 8) | (arr[2] << 16) | (arr[3] << 24);
     return *(float*) &i;
}

void floatToByte(byte* arr, float value)
{
     long l = *(long*) &value;
     arr[0] = l & 0x00FF;
     arr[1] = (l >> 8) & 0x00FF;
     arr[2] = (l >> 16) & 0x00FF;
     arr[3] = l >> 24;
}

