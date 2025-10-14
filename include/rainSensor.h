// #define DISPLAY_WIDTH 73
// #define DISPLAY_HEIGHT 41
 
//The pin definitions below are for the Seeed Studio XIAO ESP32C3 board
//They seem to be different from the ESP32 Dev board
#define SENSOR_POWER_PIN D4 //This pin powers the sensor on and off
#define SENSOR_PIN_A0 2 //The sensor is connected to ADC0
#define VCC_PIN_A1 3 //The Vcc divider is connected to ADC4
#define SOLAR_PANEL_PIN_A2 4 //The solar panel voltage divider is connected to ADC2
#define BATTERY_PIN_A3 5 //The raw battery voltage divider is connected to ADC3
#define MAX_SOLAR_PANEL_VOLTAGE 7000 //millivolts
#define MAX_BATTERY_VOLTAGE 4200 //millivolts

#define DEFAULT_RAIN_THRESHOLD 3000  // Default threshold value for rain detection
#define DISPLAY_TIMEOUT 10000 // turn off display after this many seconds of inactivity
#define DEFAULT_RAIN_CHECK_INTERVAL 15 //seconds between rain checks
#define DEFAULT_STATUS_REPORT_INTERVAL 5 //60*60 //maximum seconds between dry time status reports

#define VALID_SETTINGS_FLAG 0xDAB0
#define LED_ON LOW
#define LED_OFF HIGH
#define SSID_SIZE 50
#define PASSWORD_SIZE 50
#define ADDRESS_SIZE 30
#define USERNAME_SIZE 50
#define MAX_COMMAND_SIZE 50 // incoming command names are all smaller than this
#define MQTT_CLIENTID_SIZE 25
#define MQTT_TOPIC_SIZE 150
#define MQTT_TOPIC_SUFFIX_SIZE 15
#define MQTT_TOPIC_RAIN "raining" //topic suffix for the rain status
#define MQTT_TOPIC_VALUE "value" //topic suffix for the sensor reading
#define MQTT_TOPIC_VCC "vcc"
#define MQTT_TOPIC_BATTERY "battery"
#define MQTT_TOPIC_SOLAR_PANEL "solar"
#define MQTT_TOPIC_ANALOG "analog"
#define MQTT_TOPIC_RSSI "rssi"
#define MQTT_TOPIC_SNR "snr"
#define MQTT_TOPIC_FREE_HEAP "freeHeap"
#define MQTT_TOPIC_HEAP_FRAGMENTATION "heapFrag"
#define MQTT_TOPIC_MAX_FREE_BLOCK_SIZE "maxBlockSize"
#define MQTT_CLIENT_ID_ROOT "RainSensor"
#define MQTT_TOPIC_COMMAND_REQUEST "command"
#define MQTT_PAYLOAD_SETTINGS_COMMAND "settings" //show all user accessable settings
#define MQTT_PAYLOAD_RESET_PULSE_COMMAND "resetPulseCounter" //reset the pulse counter to zero
#define MQTT_PAYLOAD_REBOOT_COMMAND "reboot" //reboot the controller
#define MQTT_PAYLOAD_VERSION_COMMAND "version" //show the version number
#define MQTT_PAYLOAD_STATUS_COMMAND "status" //show the most recent flow values
#define MQTT_PAYLOAD_ARMED_STATUS "armed" //device has not triggered
#define MQTT_PAYLOAD_TRIPPED_STATUS "tripped" //device has triggered
#define MQTT_MAX_INCOMING_PAYLOAD_SIZE 100 //incoming MQTT message should never be this big
#define DEFAULT_EVENT_PERIOD 30 //minutes to wait after rain event before resuming status reports
#define JSON_STATUS_SIZE SSID_SIZE+PASSWORD_SIZE+USERNAME_SIZE+MQTT_TOPIC_SIZE+ADDRESS_SIZE+(MQTT_TOPIC_SUFFIX_SIZE*2)+250 //+250 for associated field names, etc
#define PUBLISH_DELAY 400 //milliseconds to wait after publishing to MQTT to allow transaction to finish
#define WIFI_TIMEOUT_SECONDS 30 // give up on wifi after this long
#define FULL_BATTERY_COUNT 3686 //raw A0 count with a freshly charged 18650 lithium battery 
#define FULL_BATTERY_VOLTS 412 //4.12 volts for a fully charged 18650 lithium battery 
#define ONE_HOUR 3600000 //milliseconds
#define STANDALONE_SSID "monitor" //SSID to use when in soft AP mode
#define STAY_AWAKE_SECONDS 30 //stay awake for this many seconds when receiving serial commands

#define ADC_ONESHOT_FORCE_USE_ADC2_ON_C3 //Needed to use ADC2 on the C3

// Function prototypes (generated from src/main.cpp)
char* generateMqttClientId(char* mqttId);
void initializeSettings();
void showSettings();
String getConfigCommand();
bool processCommand(String cmd);
void incomingSerialData();
void checkForCommand();
bool report(uint16_t value);
boolean publish(char* topic, const char* reading, boolean retain);
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length);
void reconnectToBroker();
void showSub(char* topic, bool subgood);
bool checkString(const char* str);
bool settingsSanityCheck();
boolean saveSettings();
void loadSettings();
void connectToWiFi();
void show(const char *s);
//voltages readSensors();
void initSerial();
void initDisplay();
void initSettings();
void initPorts();
void setup(void);
void loop(void);

