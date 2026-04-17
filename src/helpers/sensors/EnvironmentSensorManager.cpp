#include "EnvironmentSensorManager.h"
#if ENV_PIN_SDA && ENV_PIN_SCL
#define TELEM_WIRE &Wire1  // Use Wire1 as the I2C bus for Environment Sensors
#else
#define TELEM_WIRE &Wire  // Use default I2C bus for Environment Sensors
#endif



#if ENV_INCLUDE_SHTC3
#include <Adafruit_SHTC3.h>
static Adafruit_SHTC3 SHTC3;
#endif

#if ENV_INCLUDE_RAK12035
// inicializar librerias
// Wire ya esta incluida
// si necesitamos seesaw la incluiremos
#include <RAK12035_SoilMoisture.h>
RAK12035 sensor;
// valores de calibracion no confirmados
uint16_t zero_val = 73;
uint16_t hundred_val = 250;
#endif


#if ENV_INCLUDE_GPS && defined(RAK_BOARD) && !defined(RAK_WISMESH_TAG)
#define RAK_WISBLOCK_GPS
#endif

#ifdef RAK_WISBLOCK_GPS
static uint32_t gpsResetPin = 0;
static bool i2cGPSFlag = false;
static bool serialGPSFlag = false;
#define TELEM_RAK12500_ADDRESS   0x42     //RAK12500 Ublox GPS via i2c
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
static SFE_UBLOX_GNSS ublox_GNSS;

class RAK12500LocationProvider : public LocationProvider {
  long _lat = 41.4534663;
  long _lng = 2.1856306;
  long _alt = 0;
  int _sats = 0;
  long _epoch = 0;
  bool _fix = false;
public:
  long getLatitude() override { return _lat; }
  long getLongitude() override { return _lng; }
  long getAltitude() override { return _alt; }
  long satellitesCount() override { return _sats; }
  bool isValid() override { return _fix; }
  long getTimestamp() override { return _epoch; }
  void sendSentence(const char * sentence) override { }
  void reset() override { }
  void begin() override { }
  void stop() override { }
  void loop() override {
    if (ublox_GNSS.getGnssFixOk(8)) {
      _fix = true;
      _lat = ublox_GNSS.getLatitude(2) / 10;
      _lng = ublox_GNSS.getLongitude(2) / 10;
      _alt = ublox_GNSS.getAltitude(2);
      _sats = ublox_GNSS.getSIV(2);
    } else {
      _fix = false;
    }
    _epoch = ublox_GNSS.getUnixEpoch(2);
  }
  bool isEnabled() override { return true; }
};

static RAK12500LocationProvider RAK12500_provider;
#endif

bool EnvironmentSensorManager::begin() {
  #if ENV_INCLUDE_GPS
  #ifdef RAK_WISBLOCK_GPS
  rakGPSInit();   //probe base board/sockets for GPS
  #else
  initBasicGPS();
  #endif
  #endif

  #if ENV_PIN_SDA && ENV_PIN_SCL
    #ifdef NRF52_PLATFORM
  Wire1.setPins(ENV_PIN_SDA, ENV_PIN_SCL);
  Wire1.setClock(100000);
  Wire1.begin();
    #else
  Wire1.begin(ENV_PIN_SDA, ENV_PIN_SCL, 100000);
    #endif
  MESH_DEBUG_PRINTLN("Second I2C initialized on pins SDA: %d SCL: %d", ENV_PIN_SDA, ENV_PIN_SCL);
  #endif

  #if ENV_INCLUDE_RAK12035
  // inicializar sensor
	Wire.begin();

	// Initialize sensor
	sensor.begin();
  	// Get sensor firmware version
	uint8_t data = 0;
	sensor.get_sensor_version(&data);
	Serial.print("Sensor Firmware version: ");
	Serial.println(data, HEX);
	Serial.println();

	// Set the calibration values
	// Reading the saved calibration values from the sensor.
	sensor.get_dry_cal(&zero_val);
	sensor.get_wet_cal(&hundred_val);
	Serial.printf("Dry calibration value is %d\n", zero_val);
	Serial.printf("Wet calibration value is %d\n", hundred_val);
  #endif

  #if ENV_INCLUDE_SHTC3
  if (SHTC3.begin(TELEM_WIRE)) {
    MESH_DEBUG_PRINTLN("Found sensor: SHTC3");
    MESH_DEBUG_PRINTLN("FUNCIONA");
    SHTC3_initialized = true;
  } else {
    MESH_DEBUG_PRINTLN("NO FUNCIONA");
    SHTC3_initialized = false;
    MESH_DEBUG_PRINTLN("SHTC3 was not found at I2C address %02X", 0x70);
  }
  #endif

  return true;
}

bool EnvironmentSensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  next_available_channel = TELEM_CHANNEL_SELF + 1;

  if (requester_permissions & TELEM_PERM_LOCATION && gps_active) {
    telemetry.addGPS(TELEM_CHANNEL_SELF, node_lat, node_lon, node_altitude); // allow lat/lon via telemetry even if no GPS is detected
  }

  if (requester_permissions & TELEM_PERM_ENVIRONMENT) {

    #if ENV_INCLUDE_RAK12035
    // leer sensor
    // Read capacitance
    uint16_t capacitance = 0;
    sensor.get_sensor_capacitance(&capacitance);
    Serial.print("Soil Moisture Capacitance: ");
    Serial.println(capacitance);

    // Read moisture in %
    // after calibration, we get the Capacitance in air and in water, like zero_val and B. zero_val means 0% and B means 100%.
    // So the humidity is humidity =  (Capacitance-A) / ((B-A)/100.0)
    uint8_t moisture = 0;
    sensor.get_sensor_moisture(&moisture);
    Serial.print("Soil Moisture humidity(%): ");
    Serial.println(moisture);

    // Read temperature
    uint16_t temperature = 0;
    sensor.get_sensor_temperature(&temperature);
    Serial.print("Soil Moisture Temperatur: ");
    Serial.print(temperature / 10);
    Serial.println(" *C");
    delay(1000);

    telemetry.addTemperature(TELEM_CHANNEL_SELF, temperature);
    telemetry.addRelativeHumidity(TELEM_CHANNEL_SELF, moisture);


    #endif 

    #if ENV_INCLUDE_SHTC3
/*    if (SHTC3_initialized) {
      sensors_event_t humidity, temp;
      SHTC3.getEvent(&humidity, &temp);


      telemetry.addTemperature(TELEM_CHANNEL_SELF, temp.temperature);
      telemetry.addRelativeHumidity(TELEM_CHANNEL_SELF, humidity.relative_humidity);
    }
     


    */
      telemetry.addTemperature(TELEM_CHANNEL_SELF, 25.0);
      telemetry.addRelativeHumidity(TELEM_CHANNEL_SELF, 96.0);


    #endif



  }

  return true;
}


int EnvironmentSensorManager::getNumSettings() const {
  int settings = 0;
  #if ENV_INCLUDE_GPS
    if (gps_detected) settings++;  // only show GPS setting if GPS is detected
  #endif
  return settings;
}

const char* EnvironmentSensorManager::getSettingName(int i) const {
  int settings = 0;
  #if ENV_INCLUDE_GPS
    if (gps_detected && i == settings++) {
      return "gps";
    }
  #endif
  // convenient way to add params (needed for some tests)
//  if (i == settings++) return "param.2";
  return NULL;
}

const char* EnvironmentSensorManager::getSettingValue(int i) const {
  int settings = 0;
  #if ENV_INCLUDE_GPS
    if (gps_detected && i == settings++) {
      return gps_active ? "1" : "0";
    }
  #endif
  // convenient way to add params ...
//  if (i == settings++) return "2";
  return NULL;
}

bool EnvironmentSensorManager::setSettingValue(const char* name, const char* value) {
  #if ENV_INCLUDE_GPS
  if (gps_detected && strcmp(name, "gps") == 0) {
    if (strcmp(value, "0") == 0) {
      stop_gps();
    } else {
      start_gps();
    }
    return true;
  }
  if (strcmp(name, "gps_interval") == 0) {
    uint32_t interval_seconds = atoi(value);
    if (interval_seconds > 0) {
      gps_update_interval_sec = interval_seconds;
    } else {
      gps_update_interval_sec = 1;  // Default to 1 second if 0
    }
    return true;
  }
  #endif
  return false;  // not supported
}

#if ENV_INCLUDE_GPS

void EnvironmentSensorManager::initBasicGPS() {

  Serial1.setPins(PIN_GPS_TX, PIN_GPS_RX);

  #ifdef GPS_BAUD_RATE
  Serial1.begin(GPS_BAUD_RATE);
  #else
  Serial1.begin(9600);
  #endif

  // Try to detect if GPS is physically connected to determine if we should expose the setting
  _location->begin();
  _location->reset();

  #ifndef PIN_GPS_EN
    MESH_DEBUG_PRINTLN("No GPS wake/reset pin found for this board. Continuing on...");
  #endif

  // Give GPS a moment to power up and send data
  delay(1000);

  // We'll consider GPS detected if we see any data on Serial1
#ifdef ENV_SKIP_GPS_DETECT
  gps_detected = true;
#else
  gps_detected = (Serial1.available() > 0);
#endif

  if (gps_detected) {
    MESH_DEBUG_PRINTLN("GPS detected");
    #ifdef PERSISTANT_GPS
      gps_active = true;
      return;
    #endif
  } else {
    MESH_DEBUG_PRINTLN("No GPS detected");
  }
  _location->stop();
  gps_active = false; //Set GPS visibility off until setting is changed
}

// gps code for rak might be moved to MicroNMEALoactionProvider
// or make a new location provider ...
#ifdef RAK_WISBLOCK_GPS
void EnvironmentSensorManager::rakGPSInit(){

  Serial1.setPins(PIN_GPS_TX, PIN_GPS_RX);

  #ifdef GPS_BAUD_RATE
  Serial1.begin(GPS_BAUD_RATE);
  #else
  Serial1.begin(9600);
  #endif

  //search for the correct IO standby pin depending on socket used
  if(gpsIsAwake(WB_IO2)){
  //  MESH_DEBUG_PRINTLN("RAK base board is RAK19007/10");
  //  MESH_DEBUG_PRINTLN("GPS is installed on Socket A");
  }
  else if(gpsIsAwake(WB_IO4)){
  //  MESH_DEBUG_PRINTLN("RAK base board is RAK19003/9");
  //  MESH_DEBUG_PRINTLN("GPS is installed on Socket C");
  }
  else if(gpsIsAwake(WB_IO5)){
  //  MESH_DEBUG_PRINTLN("RAK base board is RAK19001/11");
  //  MESH_DEBUG_PRINTLN("GPS is installed on Socket F");
  }
  else{
    MESH_DEBUG_PRINTLN("No GPS found");
    gps_active = false;
    gps_detected = false;
    Serial1.end();
    return;
  }

  #ifndef FORCE_GPS_ALIVE // for use with repeaters, until GPS toggle is implimented
  //Now that GPS is found and set up, set to sleep for initial state
  stop_gps();
  #endif
}

bool EnvironmentSensorManager::gpsIsAwake(uint8_t ioPin){

  //set initial waking state
  pinMode(ioPin,OUTPUT);
  digitalWrite(ioPin,LOW);
  delay(500);
  digitalWrite(ioPin,HIGH);
  delay(500);

  //Try to init RAK12500 on I2C
  if (ublox_GNSS.begin(Wire) == true){
    MESH_DEBUG_PRINTLN("RAK12500 GPS init correctly with pin %i",ioPin);
    ublox_GNSS.setI2COutput(COM_TYPE_UBX);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_IMES);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS);
    ublox_GNSS.setMeasurementRate(1000);
    ublox_GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
    gpsResetPin = ioPin;
    i2cGPSFlag = true;
    gps_active = true;
    gps_detected = true;

    _location = &RAK12500_provider;
    return true;
  } else if (Serial1.available()) {
    MESH_DEBUG_PRINTLN("Serial GPS init correctly and is turned on");
    if(PIN_GPS_EN){
      gpsResetPin = PIN_GPS_EN;
    }
    serialGPSFlag = true;
    gps_active = true;
    gps_detected = true;
    return true;
  }

  pinMode(ioPin, INPUT);
  MESH_DEBUG_PRINTLN("GPS did not init with this IO pin... try the next");
  return false;
}
#endif

void EnvironmentSensorManager::start_gps() {
  gps_active = true;
  #ifdef RAK_WISBLOCK_GPS
    pinMode(gpsResetPin, OUTPUT);
    digitalWrite(gpsResetPin, HIGH);
    return;
  #endif

  _location->begin();
  _location->reset();

#ifndef PIN_GPS_EN
  MESH_DEBUG_PRINTLN("Start GPS is N/A on this board. Actual GPS state unchanged");
#endif
}

void EnvironmentSensorManager::stop_gps() {
  gps_active = false;
  #ifdef RAK_WISBLOCK_GPS
    pinMode(gpsResetPin, OUTPUT);
    digitalWrite(gpsResetPin, LOW);
    return;
  #endif

  _location->stop();

  #ifndef PIN_GPS_EN
  MESH_DEBUG_PRINTLN("Stop GPS is N/A on this board. Actual GPS state unchanged");
  #endif
}

void EnvironmentSensorManager::loop() {
  static long next_gps_update = 0;

  #if ENV_INCLUDE_GPS
  if (gps_active) {
    _location->loop();
  }
  if (millis() > next_gps_update) {

    if(gps_active){
    #ifdef RAK_WISBLOCK_GPS
    if ((i2cGPSFlag || serialGPSFlag) && _location->isValid()) {
      node_lat = ((double)_location->getLatitude())/1000000.;
      node_lon = ((double)_location->getLongitude())/1000000.;
      MESH_DEBUG_PRINTLN("lat %f lon %f", node_lat, node_lon);
      node_altitude = ((double)_location->getAltitude()) / 1000.0;
      MESH_DEBUG_PRINTLN("lat %f lon %f alt %f", node_lat, node_lon, node_altitude);
    }
    #else
    if (_location->isValid()) {
      node_lat = ((double)_location->getLatitude())/1000000.;
      node_lon = ((double)_location->getLongitude())/1000000.;
      MESH_DEBUG_PRINTLN("lat %f lon %f", node_lat, node_lon);
      node_altitude = ((double)_location->getAltitude()) / 1000.0;
      MESH_DEBUG_PRINTLN("lat %f lon %f alt %f", node_lat, node_lon, node_altitude);
    }
    #endif
    }
    next_gps_update = millis() + (gps_update_interval_sec * 1000);
  }
  #endif
}
#endif
