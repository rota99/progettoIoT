/*
  This example creates a BLE peripheral with a Heart Rate Service

  The circuit:
  - Arduino Nano 33 BLE / BLE Sense
  - Arduino Nano 33 IoT
  - Arduino Nano RP2040 Connect
  

  You can use a generic BLE central app, like BLE Scanner (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.
  Silicon Labs EFR Connect and Infineon/Cypress CySmart can decode Heart Rate Service.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------

// https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.heart_rate_measurement.xml
// https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.heart_rate_control_point.xml
// https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.body_sensor_location.xml

#define BLE_UUID_HEART_RATE_SERVICE               "180D"
#define BLE_UUID_HEART_RATE_MEASURMENT            "2A37"
#define BLE_UUID_BODY_SENSOR_LOCATION             "2A38"
#define BLE_UUID_HEART_RATE_CONTROL_POINT         "2A39"

//----------------------------------------------------------------------------------------------------------------------
// BLE Heart Rate Measurment
//----------------------------------------------------------------------------------------------------------------------

// Constants for flags in Heart Rate Measurement (see XML link)
#define HRM_VALUE_FORMAT_8BIT                     0
#define HRM_VALUE_FORMAT_16BIT                    1
#define HRM_SENSOR_CONTACT_NOT_SUPPORTED          ( 0 << 1 )
#define HRM_SENSOR_CONTACT_NOT_DETECTED           ( 2 << 1 )
#define HRM_SENSOR_CONTACT_DETECTED               ( 3 << 1 )
#define HRM_ENERGY_EXPENDED_NOT_PRESENT           ( 0 << 3 )
#define HRM_ENERGY_EXPENDED_PRESENT               ( 1 << 3 )


enum { BODY_SENSOR_LOCATION_OTHER = 0,
       BODY_SENSOR_LOCATION_CHEST,
       BODY_SENSOR_LOCATION_WRIST,
       BODY_SENSOR_LOCATION_FINGER,
       BODY_SENSOR_LOCATION_HAND,
       BODY_SENSOR_LOCATION_EAR_LOBE,
       BODY_SENSOR_LOCATION_FOOT
     };


typedef struct __attribute__( ( packed ) )
{
  uint8_t flags;
  uint8_t heartRate;
} heart_rate_measurment_t;


union heart_rate_measurment_u
{
  struct __attribute__( ( packed ) )
  {
    heart_rate_measurment_t values;
  };
  uint8_t bytes[ sizeof( heart_rate_measurment_t ) ];
};

union heart_rate_measurment_u heartRate = { .values = { .flags = 0, .heartRate = 0 } };

//----------------------------------------------------------------------------------------------------------------------
// BLE
//----------------------------------------------------------------------------------------------------------------------

#define BLE_DEVICE_NAME                           "Arduino HRM"
#define BLE_LOCAL_NAME                            "Arduino HRM"

BLEService heartRateService( BLE_UUID_HEART_RATE_SERVICE );
BLECharacteristic heartRateCharacteristic( BLE_UUID_HEART_RATE_MEASURMENT, BLERead | BLENotify, sizeof heartRate.bytes );
BLEUnsignedCharCharacteristic bodySensorLocationCharacteristic( BLE_UUID_BODY_SENSOR_LOCATION, BLERead );

//----------------------------------------------------------------------------------------------------------------------
// APP & I/O
//----------------------------------------------------------------------------------------------------------------------

#define BLE_LED_PIN                               LED_BUILTIN
#define HEART_BEAT_PIN                            A0
#define SENSOR_UPDATE_INTERVAL                    (1000)

bool ongoing = false;


typedef struct __attribute__( ( packed ) )
{
  uint8_t heartRate;
  bool sensorContact = false;
  bool updated = false;
} sensor_data_t;

sensor_data_t sensorData;


void setup()
{
  Serial.begin( 9600 );
//  while ( !Serial );

  Serial.println( "BLE Example Heart Rate Service" );

  pinMode( BLE_LED_PIN, OUTPUT );
  pinMode( HEART_BEAT_PIN, INPUT );
  analogReadResolution( 10 );

  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("bpm\tX\tY\tZ");

  if ( !setupBleMode() )
  {
    Serial.println( "Failed to initialize BLE!" );
    while ( 1 );
  }
  else
  {
    Serial.println( "BLE initialized. Waiting for clients to connect." );
  }
}


void loop()
{
  if(ongoing) {
    bleTask();
    
    if ( sensorTask() )
    {
      printTask();
    } else {
      printTask();
    }


    float x, y, z;

    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);
      Serial.print(x);
      Serial.print(',');
      Serial.print(y);
      Serial.print(',');
      Serial.println(z);    
    }

    delay(200);
  }

  if(Serial.available() > 0) {
      String receivedStr = "";

      while(Serial.available() > 0) {
        receivedStr += char(Serial.read());

        if(receivedStr == "0") {
          //stop reading values
          ongoing = 0;
        } else if(receivedStr == "1") {
          ongoing = 1;
        } else {
          //rischio per l'atleta
          ongoing = 0;

          for(int i = 0; i < 100; i++) {
            digitalWrite( BLE_LED_PIN, HIGH );
            delay(100);
            digitalWrite( BLE_LED_PIN, LOW );
            delay(100);
          }
        }
      }    
    }
  
}


void printTask()
{
  //Serial.print( "HR: " );
  Serial.print( sensorData.heartRate );
  Serial.print(',');
  //Serial.println( " bpm" );
}


bool sensorTask()
{
  static uint32_t previousMillis = 0;

  uint32_t currentMillis = millis();
  if ( currentMillis - previousMillis < SENSOR_UPDATE_INTERVAL )
  {
    return false;
  }
  previousMillis = currentMillis;

  uint16_t heartRate = analogRead( HEART_BEAT_PIN );
  sensorData.heartRate = ( uint8_t ) map( heartRate, 0, 1023, 40, 210 );

  if ( sensorData.heartRate < 60 )
  {
    sensorData.sensorContact = false;
  }
  else
  {
    sensorData.sensorContact = true;
  }
  sensorData.updated = true;
  return true;
}


bool setupBleMode()
{
  if ( !BLE.begin() )
  {
    return false;
  }

  // set advertised local name and service UUID
  BLE.setDeviceName( BLE_DEVICE_NAME );
  BLE.setLocalName( BLE_LOCAL_NAME );
  BLE.setAdvertisedService( heartRateService );

  // BLE add characteristics
  heartRateService.addCharacteristic( heartRateCharacteristic );
  heartRateService.addCharacteristic( bodySensorLocationCharacteristic );

  // add service
  BLE.addService( heartRateService );

  // set the initial value for the characeristic
  heartRateCharacteristic.writeValue( heartRate.bytes, sizeof heartRate.bytes );
  bodySensorLocationCharacteristic.writeValue( BODY_SENSOR_LOCATION_WRIST );

  // set BLE event handlers
  BLE.setEventHandler( BLEConnected, blePeripheralConnectHandler );
  BLE.setEventHandler( BLEDisconnected, blePeripheralDisconnectHandler );
  
  // start advertising
  BLE.advertise();

  return true;
}


void bleTask()
{
  const uint32_t BLE_UPDATE_INTERVAL = 10;
  static uint32_t previousMillis = 0;

  uint32_t currentMillis = millis();
  if ( currentMillis - previousMillis >= BLE_UPDATE_INTERVAL )
  {
    previousMillis = currentMillis;
    BLE.poll();
  }

  if ( sensorData.updated )
  {
    heartRate.values.heartRate = sensorData.heartRate;

    if ( sensorData.sensorContact )
    {
      heartRate.values.flags = ( heartRate.values.flags & 0b11111001 ) | HRM_SENSOR_CONTACT_DETECTED;
    }
    else
    {
      heartRate.values.flags = ( heartRate.values.flags & 0b11111001 ) | HRM_SENSOR_CONTACT_NOT_DETECTED;
    }

    heartRateCharacteristic.writeValue( heartRate.bytes, sizeof heartRate.bytes );
    sensorData.updated = false;
  }
}


void blePeripheralConnectHandler( BLEDevice central )
{
  digitalWrite( BLE_LED_PIN, HIGH );
  Serial.print( F( "Connected to central: " ) );
  Serial.println( central.address() );
}


void blePeripheralDisconnectHandler( BLEDevice central )
{
  digitalWrite( BLE_LED_PIN, LOW );
  Serial.print( F( "Disconnected from central: " ) );
  Serial.println( central.address() );
}