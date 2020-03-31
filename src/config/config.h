const int RS485_SEND_MODE = HIGH;
const int RS485_RECV_MODE = LOW;

#ifdef SG_SENSOR_V1
const int AIR_SENSOR_DATA_PIN = 2;
const int AIR_SENSOR_CLK_PIN = 3;

const int SOIL_SENSOR_DATA_PIN = 6;
const int SOIL_SENSOR_CLK_PIN = 7;
const int SG_STATION_DIR_PIN = 4;

const int PAR_PIN = A0;

const int SG_STATION_RX = 8;
const int SG_STATION_TX = 9;
const String SG_VERSION = "v1";
#endif

#ifdef SG_SENSOR_V2
const int AIR_SENSOR_DATA_PIN = 5;
const int AIR_SENSOR_CLK_PIN = 6;

const int SOIL_SENSOR_DATA_PIN = 2;
const int SOIL_SENSOR_CLK_PIN = 3;

const int PAR_PIN = A3;

const int SG_STATION_RX = 9;
const int SG_STATION_TX = 7;
const int SG_STATION_DIR_PIN = 8;
const String SG_VERSION = "v2_lora";
#endif
