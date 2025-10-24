#include <Arduino.h>
//#include <U8g2lib.h>
#include "driver/adc.h"
#include <Wire.h>
#include <math.h>    
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <EEPROM.h>
#include "esp_sleep.h"
#include "rainSensor.h"

#define VERSION "25.05.17.0"  //remember to update this after every change! YY.MM.DD.REV

//U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE, 6, 5); // special tiny screen
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// GPIO5 on the ESP32-C3 maps to ADC2 Channel 0
const adc2_channel_t ADC2_BATTERY_CHANNEL = ADC2_CHANNEL_0;

// The variables below will be persisted in RTC memory so they are not lost during deep sleep
// See: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html#deep-sleep
// and https://docs.platformio.org/en/latest/platforms/espressif32.html#using-rtc-memory
// Note that only certain types can be stored in RTC memory.  See the first link above for details.
// Also, the total size of all RTC variables cannot exceed 8K bytes.
// Variables in RTC memory are not initialized on boot, so they are initialized here.
RTC_NOINIT_ATTR bool rainEventReported; //true if we have reported the current rain event
RTC_NOINIT_ATTR int32_t statusReportTime; //when this gets to zero send the next status report
RTC_NOINIT_ATTR int32_t dryEventReported; //true if we have reported the end of the current rain event
// These are the settings that get stored in EEPROM.  They are all in one struct which
// makes it easier to store and retrieve.
typedef struct 
  {
  unsigned int validConfig=0; 
  char ssid[SSID_SIZE] = "";
  char wifiPassword[PASSWORD_SIZE] = "";
  char mqttBrokerAddress[ADDRESS_SIZE]=""; //default
  int mqttBrokerPort=1883;
  char mqttUsername[USERNAME_SIZE]="";
  char mqttPassword[PASSWORD_SIZE]="";
  char mqttTopicRoot[MQTT_TOPIC_SIZE]="";
  char mqttClientId[MQTT_CLIENTID_SIZE]=""; //will be the same across reboots
  bool debug=true;
  bool reportEveryWake=false; //report status every wake cycle
  char address[ADDRESS_SIZE]=""; //static address for this device
  char netmask[ADDRESS_SIZE]=""; //size of network
  int32_t reportInterval=DEFAULT_STATUS_REPORT_INTERVAL; //How many seconds of dry before making status report
  uint16_t rainThreshold=DEFAULT_RAIN_THRESHOLD; //Readings below this value indicate rain
  int32_t rainCheckInterval=DEFAULT_RAIN_CHECK_INTERVAL; //How many seconds to wait between rain checks
  } conf;
conf settings; //all settings in one struct makes it easier to store in EEPROM
boolean settingsAreValid=false;

// I need to read these analog values before turning on the WiFi because the WiFi interferes with ADC2 (A3)
typedef struct
  {
  uint16_t rainSensorValue; //raw value from the rain sensor
  uint16_t batteryVoltage; //in millivolts
  uint16_t solarPanelVoltage; //in millivolts
  uint16_t vcc; //the voltage powering the ESP32 in millivolts
  } voltages;
voltages readings;

IPAddress ip;
IPAddress mask;

String commandString = "";     // a String to hold incoming commands from serial
bool commandComplete = false;  // goes true when enter is pressed
ulong bedtime = 0; //time to go to sleep

//Generate an MQTT client ID.  This should not be necessary very often
char* generateMqttClientId(char* mqttId)
  {
  strcpy(mqttId,MQTT_CLIENT_ID_ROOT);
  strcat(mqttId, String(random(0xffff), HEX).c_str());
  if (settings.debug)
    {
    Serial.print("New MQTT userid is ");
    Serial.println(mqttId);
    }
  return mqttId;
  }

void initializeSettings()
  {
  settings.validConfig=0; 
  strcpy(settings.ssid,"");
  strcpy(settings.wifiPassword,"");
  strcpy(settings.mqttBrokerAddress,""); //default
  settings.mqttBrokerPort=1883;
  strcpy(settings.mqttUsername,"");
  strcpy(settings.mqttPassword,"");
  strcpy(settings.mqttTopicRoot,"");
  settings.rainThreshold=DEFAULT_RAIN_THRESHOLD;
  settings.rainCheckInterval=DEFAULT_RAIN_CHECK_INTERVAL;
  settings.debug=true;
  settings.reportEveryWake=false;
  strcpy(settings.address,"");
  strcpy(settings.netmask,"255.255.255.0");
  settings.reportInterval=DEFAULT_STATUS_REPORT_INTERVAL;
  generateMqttClientId(settings.mqttClientId);
  }


/* Send the settings to the serial port */
void showSettings()
  {
  Serial.print("broker=<MQTT broker host name or address> (");
  Serial.print(settings.mqttBrokerAddress);
  Serial.println(")");
  Serial.print("port=<port number>   (");
  Serial.print(settings.mqttBrokerPort);
  Serial.println(")");
  Serial.print("topicroot=<topic root> (");
  Serial.print(settings.mqttTopicRoot);
  Serial.println(")  Note: must end with \"/\"");  
  Serial.print("user=<mqtt user> (");
  Serial.print(settings.mqttUsername);
  Serial.println(")");
  Serial.print("pass=<mqtt password> (");
  Serial.print(settings.mqttPassword);
  Serial.println(")");
  Serial.print("ssid=<wifi ssid> (");
  Serial.print(settings.ssid);
  Serial.println(")");
  Serial.print("wifipass=<wifi password> (");
  Serial.print(settings.wifiPassword);
  Serial.println(")");
  Serial.print("address=<Static IP address if so desired> (");
  Serial.print(settings.address);
  Serial.println(")");
  Serial.print("netmask=<Network mask to be used with static IP> (");
  Serial.print(settings.netmask);
  Serial.println(")");
  Serial.print("rainthreshold=<Value (0-4095) below this is 'raining'> (");
  Serial.print(settings.rainThreshold);
  Serial.println(")");
  Serial.print("debug=<1|0> (");
  Serial.print(settings.debug);
  Serial.println(")");
  Serial.print("reportinterval=<seconds between regular status reports>   (");
  Serial.print(settings.reportInterval);
  Serial.println(")");
  Serial.print("raincheckinterval=<seconds between rain checks>   (");
  Serial.print(settings.rainCheckInterval);
  Serial.println(")");
  Serial.print("reporteverywake=<1|0> (");
  Serial.print(settings.reportEveryWake);
  Serial.println(")");
  
  Serial.print("MQTT Client ID is ");
  Serial.println(settings.mqttClientId);
  Serial.print("Address is ");
  Serial.println(wifiClient.localIP());

  Serial.println("\n*** Use \"resetmqttid=yes\" to reset all settings  ***");
  Serial.println("*** Use \"factorydefaults=yes\" to reset all settings  ***\n");
  
  Serial.print("\nSettings are ");
  Serial.println(settingsAreValid?"valid.":"incomplete.");
  } 

  /*
 * Check for configuration input via the serial port.  Return a null string 
 * if no input is available or return the complete line otherwise.
 */
String getConfigCommand()
  {
  if (commandComplete) 
    {
    Serial.println(commandString);
    String newCommand=commandString;
    if (newCommand.length()==0)
      newCommand='\n'; //to show available commands

    commandString = "";
    commandComplete = false;
    return newCommand;
    }
  else return "";
  }

bool processCommand(String cmd)
  {
  bool commandFound=true; //saves a lot of code
  char nme[MAX_COMMAND_SIZE]; //shouldn't get any commands larger than this
  const char *str=cmd.c_str();
  char *val=NULL;
  char *nme_t=strtok((char *)str,"=");
  if (strlen(nme_t)<MAX_COMMAND_SIZE)
    {
    strcpy(nme,nme_t);//Don't modify c_str() pointers
    if (nme_t!=NULL)
      val=strtok(NULL,"=");
    else
      strcpy(nme,"\n"); 
    
    if (nme[0]=='\n' || nme[0]=='\r' || nme[0]=='\0') //a single cr means show current settings
      {
      showSettings();
      commandFound=false; //command not found
      }
    else
      {
      //Get rid of the carriage return
      if (val!=NULL && strlen(val)>0 && val[strlen(val)-1]==13)
        val[strlen(val)-1]=0; 

      if (val!=NULL)
        {
        if (strcmp(val,"NULL")==0) //to nullify a value, you have to really mean it
          {
          strcpy(val,"");
          }
        
        if (strcmp(nme,"broker")==0)
          {
          strcpy(settings.mqttBrokerAddress,val);
          saveSettings();
          }
        else if (strcmp(nme,"port")==0)
          {
          if (!val)
            strcpy(val,"0");
          settings.mqttBrokerPort=atoi(val);
          saveSettings();
          }
        else if (strcmp(nme,"topicroot")==0)
          {
          strcpy(settings.mqttTopicRoot,val);
          if (val[strlen(val)-1] !='/') // must end with a /
            {
            strcat(settings.mqttTopicRoot,"/");
            }
          saveSettings();
          }
        else if (strcmp(nme,"user")==0)
          {
          strcpy(settings.mqttUsername,val);
          saveSettings();
          }
        else if (strcmp(nme,"pass")==0)
          {
          strcpy(settings.mqttPassword,val);
          saveSettings();
          }
        else if (strcmp(nme,"ssid")==0)
          {
          strcpy(settings.ssid,val);
          saveSettings();
          }
        else if (strcmp(nme,"wifipass")==0)
          {
          strcpy(settings.wifiPassword,val);
          saveSettings();
          }
        else if (strcmp(nme,"address")==0)
          {
          strcpy(settings.address,val);
          saveSettings();
          }
        else if (strcmp(nme,"rainthreshold")==0)
          {
          if (atoi(val)<4096)
            {
            settings.rainThreshold=atoi(val);
            saveSettings();
            }
          else
            {
            Serial.println("Rain threshold must be between 0 and 4095");
            }
          }
        else if (strcmp(nme,"netmask")==0)
          {
          strcpy(settings.netmask,val);
          saveSettings();
          }
        else if (strcmp(nme,"debug")==0)
          {
          if (!val)
            strcpy(val,"0");
          settings.debug=atoi(val)==1?true:false;
          saveSettings();
          }
        else if (strcmp(nme,"reporteverywake")==0)
          {
          if (!val)
            strcpy(val,"0");
          settings.reportEveryWake=atoi(val)==1?true:false;
          saveSettings();
          }
        else if (strcmp(nme,"reportinterval")==0)
          {
          if (!val)
            strcpy(val,"0");
          settings.reportInterval=atoi(val);
          statusReportTime=settings.reportInterval; //reset the timer
          saveSettings();
          }
        else if (strcmp(nme,"raincheckinterval")==0)
          {
          if (!val)
            strcpy(val,"0");
          settings.rainCheckInterval=atoi(val);
          saveSettings();
          }

        else if ((strcmp(nme,"resetmqttid")==0)&& (strcmp(val,"yes")==0))
          {
          generateMqttClientId(settings.mqttClientId);
          saveSettings();
          }
        else if ((strcmp(nme,"factorydefaults")==0) && (strcmp(val,"yes")==0)) //reset all eeprom settings
          {
          Serial.println("\n*********************** Resetting EEPROM Values ************************");
          initializeSettings();
          saveSettings();
          delay(2000);
          ESP.restart();
          }
        else
          {
          showSettings();
          commandFound=false; //command not found
          }
        }
      }
    }
  else
    {
    Serial.print("Incoming command too large: ");
    Serial.println(nme_t);
    commandFound=false;
    }

  return commandFound;
  }

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void incomingSerialData() 
  {
  static bool lastCR = false; 
    {
    char inChar = (char)Serial.read(); // get the new byte
    Serial.print(inChar); //echo it back to the terminal

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it 
    if (inChar == '\n' || inChar == '\r') 
      {
      if (lastCR)     //some serial ports send both CR and LF, We want to ignore the second one
        lastCR=false;
      else
        {
        lastCR=true;
        commandComplete = true;
        }
      }
    else
      {
      lastCR=false; //in case only one of \r and \n is sent
      // add it to the inputString 
      commandString += inChar;
      }
    }
  }

void checkForCommand()
  {
  if (Serial.available())
    {
    incomingSerialData();
    String cmd=getConfigCommand();
    if (cmd.length()>0)
      {
      bedtime=millis()+STAY_AWAKE_SECONDS*1000; //stay awake for a while
      yield();
      processCommand(cmd);
      }
    }
  }

/* Read and return the VCC voltage in millivolts */
uint32_t getVcc()
  {
  int result = analogRead(VCC_PIN_A1); // read the Vcc directly
  //There is no voltage divider on Vcc, so the formula is simple
  //float vcc = result/4095.0f * 3.3f; // Calculate Vcc (in Volts); 4095 for 12 bit ADC
  result=map(result, 0, 4095, 0, 3300); //millivolts
  Serial.print("\n**************** Mapped Vcc:  ");
  Serial.println(result);
  return result;
  }

/* Read and return the battery voltage in millivolts */
uint32_t getBattery()
  {
  #define R1 470.0f // resistor values for the voltage divider
  #define R2 470.0f
  #define ADC_MAX_VALUE 4095.0f // 12-bit ADC

  // Fraction of battery voltage seen by ADC (0.5 for R1=R2)
  #define BATTERY_DIVIDER_RATIO (R2/(R1+R2)) 

  int result = analogRead(BATTERY_PIN_A3); // read the battery divider value

  // 1. Calculate the voltage at the divider tap (rawToMillivolts) using float math
  // (Result / ADC_MAX_VALUE) * ADC_MAX_VOLTAGE_MV
  float rawToMillivolts = ((float)result / ADC_MAX_VALUE) * ADC_MAX_VOLTAGE_MV; 

  // 2. Calculate the actual battery voltage (bt)
  // Voltage_Battery = Voltage_Tap / BATTERY_DIVIDER_RATIO (which is 0.5)
  float bt_float = rawToMillivolts / BATTERY_DIVIDER_RATIO; 
  
  // The final result should be returned as an integer (uint32_t), so cast it.
  uint32_t bt = (uint32_t)bt_float;

  Serial.print("\n**************** Raw Battery:  ");
  Serial.println(result);
  
  Serial.print("\n**************** Mapped Battery:  ");
  Serial.println(bt);
  return bt;
  }

/* Read and return the solar panel voltage in millivolts */
uint32_t getSolarPanel()
  {
  #define R3 680.0f
  #define R4 470.0f
  #define SOL_MAX_RAW_VOLTS MAX_SOLAR_PANEL_VOLTAGE*(R4/(R3+R4))
  #define SOL_MAX_RAW_VALUE 4095*(SOL_MAX_RAW_VOLTS)/(ADC_MAX_VOLTAGE_MV) //maximum raw ADC value we expect to see from the solar panel divider
  int result = analogRead(SOLAR_PANEL_PIN_A2); // read the battery divider value
  Serial.print("\n**************** Raw Solar Panel:  ");
  Serial.println(result);
  result=map(result, 0, SOL_MAX_RAW_VALUE, 0, MAX_SOLAR_PANEL_VOLTAGE); //millivolts
  
  Serial.print("\n**************** Mapped Solar Panel:  ");
  Serial.println(result);

  return result;
  }

/************************
 * Do the MQTT thing
 ************************/
bool report(uint16_t value)
  {
  char topic[MQTT_TOPIC_SIZE+9];
  char reading[18];
  bool ok=true;

  //publish the rain sensor reading
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_VALUE);
  sprintf(reading,"%d",value); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  //publish the rain status
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_RAIN);
  sprintf(reading,"%s",value>settings.rainThreshold?"false":"true"); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  //publish the radio strength reading while we're at it
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_RSSI);
  sprintf(reading,"%d",WiFi.RSSI()); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  //publish the supply voltage
  uint32_t milliVolts = readings.vcc; // millivolts
  float volts = (float)milliVolts / 1000.0;// Convert to Volts
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_VCC);
  sprintf(reading,"%.2f",volts); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  //publish the battery voltage
  milliVolts = readings.batteryVoltage; // millivolts
  volts = (float)milliVolts / 1000.0;// Convert to Volts
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_BATTERY);
  sprintf(reading,"%.2f",volts); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  //publish the solar panel voltage
  milliVolts = readings.solarPanelVoltage; // millivolts
  volts = (float)milliVolts / 1000.0;// Convert to Volts
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_SOLAR_PANEL);
  sprintf(reading,"%.2f",volts); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  // // Publish some memory usage info
  // uint32_t freeHeap = ESP.getFreeHeap();
  // strcpy(topic,settings.mqttTopicRoot);
  // strcat(topic,MQTT_TOPIC_FREE_HEAP);
  // sprintf(reading,"%d",freeHeap); 
  // ok=ok & publish(topic,reading,true); //retain
  // yield();

  if (settings.debug)
    {
    Serial.print("Publish ");
    Serial.println(ok?"OK":"Failed");
    }
  return ok;
  }


boolean publish(char* topic, const char* reading, boolean retain)
  {
  if (settings.debug)
    {
    Serial.print(topic);
    Serial.print(" ");
    Serial.println(reading);
    }
  boolean ok=false;
  connectToWiFi(); //just in case we're disconnected from WiFi
  reconnectToBroker(); //also just in case we're disconnected from the broker

  if (mqttClient.connected() && 
      settings.mqttTopicRoot &&
      WiFi.status()==WL_CONNECTED)
    {
    ok=mqttClient.publish(topic,reading,retain); 
    mqttClient.loop(); //check for incoming messages
    }
  else
    {
    Serial.print("Can't publish due to ");
    if (WiFi.status()!=WL_CONNECTED)
      Serial.println("no WiFi connection.");
    else if (!mqttClient.connected())
      Serial.println("not connected to broker.");
    }
  return ok;
  }



/**
 * Handler for incoming MQTT messages.  The payload is the command to perform. 
 * The MQTT message topic sent is the topic root plus the command.
 * Implemented commands are: 
 * MQTT_PAYLOAD_SETTINGS_COMMAND: sends a JSON payload of all user-specified settings
 * MQTT_PAYLOAD_REBOOT_COMMAND: Reboot the controller
 * MQTT_PAYLOAD_VERSION_COMMAND Show the version number
 * MQTT_PAYLOAD_STATUS_COMMAND Show the most recent flow values
 */
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length) 
  {
  bedtime=millis()+1000; //stay awake long enough to process the command

  if (settings.debug)
    {
    Serial.println("====================================> Callback works.");
    }
  boolean rebootScheduled=false; //so we can reboot after sending the reboot response
  char charbuf[MQTT_MAX_INCOMING_PAYLOAD_SIZE];
  if (length < MQTT_MAX_INCOMING_PAYLOAD_SIZE)
    {
    payload[length]='\0'; //this should have been done in the calling code, shouldn't have to do it here
    sprintf(charbuf,"%s",payload);
    const char* response;
    
    
    //if the command is MQTT_PAYLOAD_SETTINGS_COMMAND, send all of the settings
    if (strcmp(charbuf,MQTT_PAYLOAD_SETTINGS_COMMAND)==0)
      {
      char tempbuf[35]; //for converting numbers to strings
      char jsonStatus[JSON_STATUS_SIZE];
      
      strcpy(jsonStatus,"{");
      strcat(jsonStatus,"\"broker\":\"");
      strcat(jsonStatus,settings.mqttBrokerAddress);
      strcat(jsonStatus,"\", \"port\":");
      sprintf(tempbuf,"%d",settings.mqttBrokerPort);
      strcat(jsonStatus,tempbuf);
      strcat(jsonStatus,", \"topicroot\":\"");
      strcat(jsonStatus,settings.mqttTopicRoot);
      strcat(jsonStatus,"\", \"user\":\"");
      strcat(jsonStatus,settings.mqttUsername);
      strcat(jsonStatus,"\", \"pass\":\"");
      strcat(jsonStatus,settings.mqttPassword);
      strcat(jsonStatus,"\", \"ssid\":\"");
      strcat(jsonStatus,settings.ssid);
      strcat(jsonStatus,"\", \"wifipass\":\"");
      strcat(jsonStatus,settings.wifiPassword);
      strcat(jsonStatus,"\", \"mqttClientId\":\"");
      strcat(jsonStatus,settings.mqttClientId);
      strcat(jsonStatus,"\", \"address\":\"");
      strcat(jsonStatus,settings.address);
      strcat(jsonStatus,"\", \"netmask\":\"");
      strcat(jsonStatus,settings.netmask);
      strcat(jsonStatus,"\", \"rainthreshold\":\"");
      sprintf(tempbuf,"%d",settings.rainThreshold);
      strcat(jsonStatus,tempbuf);
      strcat(jsonStatus,"\", \"debug\":\"");
      strcat(jsonStatus,settings.debug?"true":"false");
      strcat(jsonStatus,"\", \"reporteverywake\":\"");
      strcat(jsonStatus,settings.reportEveryWake?"true":"false");
      strcat(jsonStatus,"\", \"reportinterval\":\"");
      sprintf(tempbuf,"%d",settings.reportInterval);
      strcat(jsonStatus,tempbuf);
      strcat(jsonStatus,"\", \"raincheckinterval\":");
      sprintf(tempbuf,"%d",settings.rainCheckInterval);
      strcat(jsonStatus,tempbuf);
      strcat(jsonStatus,", \"IPAddress\":\"");
      strcat(jsonStatus,wifiClient.localIP().toString().c_str());
      strcat(jsonStatus,"\",");
      strcat(jsonStatus,"\"ports\":[");
      size_t len = strlen(jsonStatus);
      if (jsonStatus[len - 1] == ',')
        jsonStatus[len - 1] = ']';   //replace the last comma to close the array
      else
        strcat(jsonStatus,"]"); //happens when port array is empty
      
      strcat(jsonStatus,"}");
      response=jsonStatus;
      }
    else if (strcmp(charbuf,MQTT_PAYLOAD_VERSION_COMMAND)==0) //show the version number
      {
      char tmp[15];
      strcpy(tmp,VERSION);
      response=tmp;
      }
    else if (strcmp(charbuf,MQTT_PAYLOAD_STATUS_COMMAND)==0) //show the latest value
      {
      report(readings.rainSensorValue);
      char tmp[25];
      strcpy(tmp,"Status report complete");
      response=tmp;
      }
    else if (strcmp(charbuf,MQTT_PAYLOAD_REBOOT_COMMAND)==0) //reboot the controller
      {
      char tmp[10];
      strcpy(tmp,"REBOOTING");
      response=tmp;
      rebootScheduled=true;
      }
    else if (processCommand(charbuf))
      {
      response="OK";
      }
    else
      {
      char badCmd[18];
      strcpy(badCmd,"(empty)");
      response=badCmd;
      }
      
    char topic[MQTT_TOPIC_SIZE];
    strcpy(topic,settings.mqttTopicRoot);
    strcat(topic,charbuf); //the incoming command becomes the topic suffix

    if (!publish(topic,response,false)) //do not retain
      Serial.println("************ Failure when publishing status response!");
      
    delay(2000); //give publish time to complete
    
    if (rebootScheduled)
      {
      ESP.restart();
      }
    }
  else
    Serial.println("Incoming MQTT message too large.");
  }


/*
 * Reconnect to the MQTT broker
 */
void reconnectToBroker() 
  {
  if (strlen(settings.mqttBrokerAddress)>0)
    {
    if (WiFi.status() != WL_CONNECTED)
      {
      Serial.println("WiFi not ready, skipping MQTT connection");
      }
    else
      {
      // Loop until we're reconnected or we give up
      int tries=20;
      while (!mqttClient.connected() && tries-->0) 
        {
        Serial.print("Attempting MQTT connection...");

        mqttClient.setBufferSize(JSON_STATUS_SIZE); //default (256) isn't big enough
        mqttClient.setKeepAlive(120); //seconds
        mqttClient.setServer(settings.mqttBrokerAddress, settings.mqttBrokerPort);
        mqttClient.setCallback(incomingMqttHandler);
        yield();

        // Attempt to connect
        if (mqttClient.connect(settings.mqttClientId,settings.mqttUsername,settings.mqttPassword))
          {
          Serial.println("connected to MQTT broker.");

          //resubscribe to the incoming message topic
          char topic[MQTT_TOPIC_SIZE];
          strcpy(topic,settings.mqttTopicRoot);
          strcat(topic,MQTT_TOPIC_COMMAND_REQUEST);
          bool subgood=mqttClient.subscribe(topic);
          showSub(topic,subgood);
          }
        else 
          {
          Serial.print("failed, rc=");
          Serial.println(mqttClient.state());
          Serial.println("Will try again in a second");
          
          // Wait a second before retrying
          // In the meantime check for input in case something needs to be changed to make it work
        //  checkForCommand(); 
          yield();
          delay(1000);
          yield();
          }
        checkForCommand();
        }
      if (!mqttClient.connected())
        {
        Serial.println("Could not connect to MQTT broker.");
        }
      else
        mqttClient.loop(); //This has to happen every so often to check for incoming messages
      }
    }
  else if (settings.debug)
    {
    Serial.println("Broker address not set, ignoring MQTT");
    }
  }

void showSub(char* topic, bool subgood)
  {
  if (settings.debug)
    {
    Serial.print("++++++Subscribing to ");
    Serial.print(topic);
    Serial.print(":");
    Serial.println(subgood);
    }
  }

/**
 * @brief Checks if a C-style string (char array) contains only
 * alphanumeric characters (a-z, A-Z, 0-9).
 *
 * @param str The null-terminated char array to check.
 * @return true if all characters are alphanumeric or the string is empty.
 * @return false if any character is not alphanumeric.
 */
bool checkString(const char* str) 
  {
    if (str == nullptr) 
      {
      return false; // A null pointer is not a valid string
      }

    // Iterate through the string until the null terminator
    for (size_t i = 0; str[i] != '\0'; ++i) 
      {
      // Cast char to unsigned char to avoid potential issues with 
      // negative char values when passed to ctype functions.
      unsigned char ch = static_cast<unsigned char>(str[i]); // Cast once
      if (!isalnum(ch) && ch != '/' && ch != '.')
        {
        return false; // Found a non-alphanumeric character
        }
      }

  // If the loop completes, all characters were alphanumeric (or the string was empty)
  return true;
  }


/*
 * Check all of the strings in the settings.   If any of the
 * character strings in the settings fail the test, it's 
 * likely that the settings are corrupt. 
*/  
bool settingsSanityCheck()
  {
  return checkString(settings.ssid)
      && checkString(settings.wifiPassword)
      && checkString(settings.mqttBrokerAddress)
      && checkString(settings.mqttUsername)
      && checkString(settings.mqttPassword)
      && checkString(settings.mqttTopicRoot)
      && checkString(settings.mqttClientId)
      && checkString(settings.address)
      && checkString(settings.netmask)
      && settings.rainThreshold<4096
      ;
  }

/*
 * Save the settings to EEPROM. Set the valid flag if everything is filled in.
 */
boolean saveSettings()
  {
  if (strlen(settings.ssid)>0 &&
      strlen(settings.wifiPassword)>0 &&
      // strlen(settings.mqttBrokerAddress)>0 &&
      // settings.mqttBrokerPort!=0 &&
      strlen(settings.mqttTopicRoot)>0 &&
      strlen(settings.mqttClientId)>0 &&
      settingsSanityCheck())
    {
    Serial.println("Settings deemed complete");
    settings.validConfig=VALID_SETTINGS_FLAG;
    settingsAreValid=true;
    }
  else
    {
    Serial.println("Settings still incomplete");
    settings.validConfig=0;
    settingsAreValid=false;
    }
    
  //The mqttClientId is not set by the user, but we need to make sure it's set  
  if (strlen(settings.mqttClientId)==0)
    {
    generateMqttClientId(settings.mqttClientId);
    }
    
  EEPROM.put(0,settings);
  if (settings.debug)
    Serial.println("Committing settings to eeprom");
  return EEPROM.commit();
  }


/*
*  Initialize the settings from eeprom and determine if they are valid
*/
void loadSettings()
  {
  EEPROM.get(0,settings);

  if (!settingsSanityCheck()) //if something is wildly off then don't run, allow setup
    {
    settings.validConfig=0;
    settingsAreValid=false;
    Serial.println(F("Settings are corrupt, marking invalid."));
    }
  else if (settings.validConfig==VALID_SETTINGS_FLAG)    //skip loading stuff if it's never been written
    {
    settingsAreValid=true;
    if (settings.debug)
      {
      Serial.println("\nLoaded configuration values from EEPROM");
      }
    }
  else
    {
    Serial.println("Skipping load from EEPROM, device not configured.");    
    settingsAreValid=false;
    }
    showSettings();
  }


 /*
 * If not connected to wifi, connect.
 */
void connectToWiFi()
  {
  if (settingsAreValid && WiFi.status() != WL_CONNECTED)
    {
    Serial.print("Attempting to connect to WPA SSID \"");
    Serial.print(settings.ssid);
    Serial.println("\"");

    WiFi.disconnect(true); // Completely reset Wi-Fi stack
    delay(100); // Small delay to ensure reset is applied
    WiFi.persistent(false); // Prevent saving to flash
    WiFi.mode(WIFI_STA); //station mode, we are only a client in the wifi world

    if (settingsAreValid)
      {      
      if (!ip.fromString(settings.address))
        {
        Serial.println("Static IP Address '"+String(settings.address)+"' is blank or not valid. Using dynamic addressing.");
        }
      else if (!mask.fromString(settings.netmask))
        {
        Serial.println("Static network mask "+String(settings.netmask)+" is not valid. Using default");
        mask.fromString("255.255.255.0");
        }
      }
    if (ip) //Use the static address
      {
      if (!WiFi.config(ip,ip,mask))
        {
        Serial.println("STA Failed to configure");
        }
      }

    unsigned long connectTimeout = millis() + WIFI_TIMEOUT_SECONDS*1000; // 30 second timeout
    WiFi.begin(settings.ssid, settings.wifiPassword);
    //delay(1000);
    unsigned long lastDotTime = millis(); // For printing dots without blocking
    while (WiFi.status() != WL_CONNECTED && millis() < connectTimeout) 
      {
      // Not yet connected
      if (millis() - lastDotTime > 500) // Print dot every 500ms, but don't block
        {
        Serial.print(".");
        lastDotTime = millis();
        yield();
        }
      checkForCommand(); // Check for input in case something needs to be changed to work
      yield();
      }

    if (WiFi.status() == WL_CONNECTED)
      {
      Serial.print("\nConnected to network with address ");
      Serial.println(WiFi.localIP());
      Serial.println();
      }
    else
      {
      Serial.println("\nFailed to connect to WiFi\n");
      }
    }
  }
 

void show(const char *s) 
  {
  // displayOffTimeMillis = millis()+DISPLAY_TIMEOUT; // reset the display timeout
  // u8g2.clearBuffer();     // clear the internal memory
  // u8g2.setCursor(0, 10);  // set cursor to top left corner
  // u8g2.print(s);          // write the string to the internal memory
  // u8g2.sendBuffer();      // transfer internal memory to the display
  }
 
voltages readSensors()
  {
  digitalWrite(SENSOR_POWER_PIN, HIGH); // turn on the power to the sensor
  delay(10); // wait for the sensor to power up
  readings.rainSensorValue = analogRead(SENSOR_PIN_A0); // take a reading from the optical sensor
  digitalWrite(SENSOR_POWER_PIN, LOW); // turn off the power to the sensor

  readings.vcc = getVcc(); // millivolts
  readings.batteryVoltage = getBattery(); // millivolts
  readings.solarPanelVoltage = getSolarPanel(); // millivolts
  
  Serial.print("\nRain sensor value: ");
  Serial.println(readings.rainSensorValue);
  Serial.print("Vcc: ");
  Serial.println(readings.vcc);
  Serial.print("Battery: ");
  Serial.println(readings.batteryVoltage);
  Serial.print("Solar Panel: ");
  Serial.println(readings.solarPanelVoltage);
  Serial.println("\n");

  return readings;
  }

void initSerial()
  {
  Serial.begin(9600);

  // Set the TX timeout to 0 milliseconds so that it will run without a serial port attached
  Serial.setTxTimeoutMs(0); 

  // Now, all subsequent Serial.print() and Serial.println() calls 
  // should be non-blocking. If the buffer is full (i.e., the PC isn't listening), 
  // the data is dropped, and the program continues immediately.  
  
  delay(100); //give time for serial to start 
  Serial.println();
  Serial.println("Serial communications established.");
  delay(5000);
  commandString.reserve(200); // reserve 200 bytes of serial buffer space for incoming command string
  }

void initDisplay()
  {
  // u8g2.begin();
  // u8g2.setContrast(255);    // set contrast to maximum
  // u8g2.setBusClock(400000); // 400kHz I2C
  // u8g2.setFont(u8g2_font_8x13_tr);
  }


void initSettings()
  {
  EEPROM.begin(sizeof(settings)); //fire up the eeprom section of flash
  loadSettings(); //set the values from eeprom 

  //show the MAC address
  Serial.print("ESP8266 MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (settings.mqttBrokerPort < 0) //then this must be the first powerup
    {
    Serial.println("\n*********************** Resetting All EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    delay(2000);
    ESP.restart();
    }
  }


void initPorts()
  {
  pinMode(SENSOR_POWER_PIN, OUTPUT);   // set the sensor power pin as an output
  digitalWrite(SENSOR_POWER_PIN, LOW); // turn off the power to the sensor until we need it

  // 1. Set 12-bit resolution for all ADC reads
  analogReadResolution(12);

  // Set all ADC ranges to to 0-3.3V
  analogSetPinAttenuation(SENSOR_PIN_A0, ADC_11db); 
  analogSetPinAttenuation(VCC_PIN_A1, ADC_11db); 
  analogSetPinAttenuation(SOLAR_PANEL_PIN_A2, ADC_11db); 
  analogSetPinAttenuation(BATTERY_PIN_A3, ADC_11db);
  }

/* The variables in RTC will persist across reboots, but we don't want them to be initialized
   on every boot.  This should ideally only be run once. */
void initRTCVars()
  {
  if (statusReportTime<(-60*60) || statusReportTime>(60*60*24)) //more than a day doesn't make sense
    {
    Serial.println("Initializing RTC variables");
    statusReportTime=settingsAreValid?settings.reportInterval:DEFAULT_STATUS_REPORT_INTERVAL; //when this gets to zero send the next status report
    rainEventReported=false; 
    dryEventReported=false;
    }
  }


void setup(void) 
  {
  initSerial();

  initSettings();
  
  initRTCVars();

  initDisplay();


  // // Set 11dB attenuation (up to ~3.3V) for the specific channel
  // esp_err_t result = adc2_config_channel_atten(ADC2_BATTERY_CHANNEL, ADC_ATTEN_DB_11);
  
  // if (result == ESP_OK)
  //   {
  //   Serial.println("ADC2 Channel configured successfully.");
  //   }
  // else
  //   {
  //   // If this fails, the force flag definitely isn't working.
  //   Serial.printf("ADC2 Configuration Failed in setup() (Error: %s)\n", esp_err_to_name(result));
  //   }





  initPorts();
  }

void loop(void) 
  {
  static bool once=true; //for testing and looping without deep sleep
  static voltages vals;
  
  if (once)
    vals=readSensors();// take a reading from all sensors before turning on the wifi
  uint16_t sensorReading = vals.rainSensorValue;

  ulong now= millis();
  
  /////////// Display stuff
 // display the new reading
  char buffer[20];
  if (settingsAreValid)
    sprintf(buffer, "%d %s", 
            sensorReading,
            sensorReading>settings.rainThreshold?"DRY":"RAIN"); 
  else
    sprintf(buffer, "%d %s", 
        sensorReading,
        sensorReading>DEFAULT_RAIN_THRESHOLD?"DRY":"RAIN");

  //show(buffer); // display the reading on the OLED screen
    
  //////////// Reporting stuff
  if (settingsAreValid)
    {
    bool raining = sensorReading < settings.rainThreshold;

    if (settings.reportEveryWake && once) //report every time we wake up
      {
      Serial.println("Reporting on every wakeup as configured.");
      statusReportTime=0; //force a report
      }

    if (raining && !rainEventReported) //we are now in a rain event and haven't reported it yet
      {
      Serial.println("Rain event started, reporting.");
      if (report(sensorReading)) //try to report the reading
        {
        rainEventReported=true; //only report once per event
        dryEventReported=false; //reset the dry event flag
        statusReportTime=settings.reportInterval; //reset the status report timer
        }
      else
        {
        Serial.println("Report failed.");
        }
      }
    else if (!raining && !dryEventReported) //we are now dry and haven't reported it yet
      {
      Serial.println("Rain event ended, reporting.");
      if (report(sensorReading)) //try to report the reading
        {
        rainEventReported=false; //reset the rain event flag
        dryEventReported=true; //only report once per event
        statusReportTime=settings.reportInterval; //reset the status report timer
        }
      }
    else if (!raining) //not raining, reset the event flag
      {
      rainEventReported=false; //ready for next event
      }
    else
      {
      dryEventReported=false; //staying dry
      }


    if (statusReportTime<=0) //time for a status report
      {
      Serial.println("Status report time expired, reporting.");
      if (report(sensorReading)) //try to report the reading
        {
        Serial.print("Resetting status report time to ");
        Serial.println(settings.reportInterval);    
        statusReportTime=settings.reportInterval; //reset the status report timer
        }
      }
    }
  else
    {
    Serial.println("Settings not valid, cannot report.");
    while(settingsAreValid==false) //wait here until settings are fixed
      {
      checkForCommand(); //check for input in case something needs to be changed to make it work
      delay(100); //don't burn up the CPU
      }
    Serial.println("Settings fixed, will try to report on next wakeup.");
    }
  
  if (once)
    {
    once=false;
    Serial.print("******************* ");
    Serial.println(buffer);
    Serial.print("StatusReportTime = ");
    Serial.println(statusReportTime);
    }

 
  checkForCommand(); //check for input in case something needs to be changed to make it work
  
  if (settingsAreValid && now>=bedtime)
    {
    Serial.print("Unadjusted status report time is ");
    Serial.println(statusReportTime);    
    statusReportTime=statusReportTime-settings.rainCheckInterval-millis()/1000; //decr the status report timer (include run time)
    Serial.print("New status report time is ");
    Serial.println(statusReportTime);
    }
  
  if (settings.debug && settingsAreValid && now>=bedtime)
    {
    Serial.println("Debug on - simulating sleep delay");
    ulong sleepTime=settings.rainCheckInterval*1000;
    sleepTime+=millis();
    while (millis()<sleepTime)
      {
      checkForCommand(); //check for input in case something needs to be changed 
      delay(100); //don't burn up the CPU
      }
    Serial.println("Simulated sleep delay complete, simulating wakeup.");
    delay(1000); //wait for serial to complete

    esp_restart(); //just restart instead of sleeping

    // esp_sleep_enable_timer_wakeup(1); //wake up immediately
    // esp_deep_sleep_start(); //sleep for zero seconds, just to simulate a wakeup
    }

  if (settingsAreValid && now>=bedtime) //time to sleep
    {
    Serial.println("Sleeping for "+String(settings.rainCheckInterval)+" seconds.");
    bedtime=millis()+1000; //just in case an MQTT message has come in
    do //stay awake while incoming serial commands are possible
      {
      checkForCommand(); //check for input in case something needs to be changed 
      mqttClient.loop(); //check for incoming messages
      } while(millis()<bedtime);
    //u8g2.setPowerSave(true); // turn off the display
    //delay(100); // wait for the display to turn off
    esp_sleep_enable_timer_wakeup(settings.rainCheckInterval*1000000); //seconds to microseconds
    esp_deep_sleep_start();
    }
  }