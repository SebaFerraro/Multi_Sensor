#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <MQ2Lib.h>
#include <Adafruit_BMP085.h>

#define RLY_I1 16
#define RLY_I2 17
#define RLY_I3 18
#define RLY_I4 19
#define DHT_PWR 4
#define DHT_DATA 23
#define DHTTYPE DHT11 
#define ANALOG_PIN_0 34
#define TOKEN_TB "5hS7riBz8EOuFzo8uGjQ"


float lpg = 0, co = 0, smoke = 0, pres=0 ;

DHT dht(DHT_DATA, DHTTYPE);
MQ2 mq2(ANALOG_PIN_0,true);
static int taskCore = 0;
int Temp=0;
int Nivel=1;
char bufferT[28]="";
int analog_value = 0;

const char* ssid     = "SSID";
const char* password = "PASS";

static int Wconectado = 0;
boolean gpioState[36] = {false};
const char* mqtt_server = "190.2.22.61";
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BMP085 bmp;

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Wconectado=1;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection.  Attempting to reconnect...");
      //WiFi.reconnect();
      Wconectado=0;
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("ESP32 station start");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("ESP32 station connected to AP");
      break;
    default:      
      Serial.println("Unhandled WiFi Event raised.");
      break;
    }
}

void Wifi_init(){ 
  WiFi.disconnect(true);
  delay(1000);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
}

String get_gpio_status() {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& data = jsonBuffer.createObject();
  data[String(RLY_I1)] = gpioState[RLY_I1] ? true : false;
  data[String(RLY_I2)] = gpioState[RLY_I2] ? true : false;
  data[String(RLY_I3)] = gpioState[RLY_I3] ? true : false;
  data[String(RLY_I4)] = gpioState[RLY_I4] ? true : false;
  char payload[256];
  data.printTo(payload, sizeof(payload));
  String strPayload = String(payload);
  Serial.print("Gpio Estado: ");
  Serial.println(strPayload);
  return strPayload;
}

void set_gpio_status(int pin, boolean enabled) {
   digitalWrite(pin, enabled ? HIGH : LOW);
   // Update GPIOs state
   gpioState[pin] = enabled;
}

void on_message(char* topic, byte* payload, unsigned int length) {
  StaticJsonBuffer<200> jsonBuffer;
  Serial.println("Mensaje Recibido");

  char json[length + 1];
  strncpy (json, (char*)payload, length);
  json[length] = '\0';

  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Mensaje: ");
  Serial.println(json);
  JsonObject& data = jsonBuffer.parseObject((char*)json);

  if (!data.success())
  {
    Serial.println("parseObject() Error");
    return;
  }

  String methodName = String((const char*)data["method"]);

  if (methodName.equals("getGpioStatus") or methodName.equals("getValue")) {
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
  } else if (methodName.equals("setGpioStatus")) {
    // Update GPIO status and reply
    set_gpio_status(data["params"]["pin"], data["params"]["enabled"]);
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
    client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
  } else if (methodName.equals("setValue")) {
    set_gpio_status(2, data["params"]);
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), data["params"]);
    client.publish("v1/devices/me/attributes",data["params"]);
  }
}

void send_mqtt(float t, float h, float hic, float co, float lpg, float smk, float pres){
  if (Wconectado == 1){
    if(!client.connected()) {
      Serial.print("Conectando ThingsBoard node ...");
      if ( client.connect("Esp32COSensor", TOKEN_TB, NULL) ) {
        Serial.println( "[DONE]" );
        int rsus=client.subscribe("v1/devices/me/rpc/request/+");
        Serial.print( "[SUBSCRIBE]" );
        Serial.println(rsus);
        client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
        Serial.print("Enviando GPIO status ...");
        Serial.println(rsus);
      } else {
        Serial.print( "[FAILED] [ rc = " );
        Serial.print( client.state() );
      }
    }  
    if (client.connected()) {
        
      String temperatura = String(t);
      String humedad = String(h);
      String stermica = String(hic);
      String sco = String(co);
      String LPG = String(lpg);
      String smoke = String(smk);
      String presi = String(pres);
      
      
      String payload = "{";
      payload += "\"temp\":";
      payload += temperatura;
      payload += ",";
      payload += "\"hum\":";
      payload += humedad;
      payload += ",";
      payload += "\"sterm\":";
      payload += stermica;
      payload += ",";
      payload += "\"co\":";
      payload += sco;
      payload += ",";
      payload += "\"lpg\":";
      payload += LPG;
      payload += ",";
      payload += "\"smoke\":";
      payload += smoke;
      payload += ",";
      payload += "\"pres\":";
      payload += presi;
      payload += "}";

      // Send payload
      char attributes[200];
      payload.toCharArray( attributes, 200 );
      int rsus=client.publish( "v1/devices/me/telemetry", attributes );
      Serial.print( "Publish : ");
      Serial.println(rsus);
      //client.publish(TEMP_TOPIC, msg);
      Serial.println( attributes );
    }
  }
}


void coreTask( void * pvParameters ){
 
    String taskMessage = "Corriendo en core ";
    taskMessage = taskMessage + xPortGetCoreID();
    Serial.println(taskMessage);
    while(true){
      if(Wconectado == 1){
          if(!client.connected()) {
              Serial.print("Conectando a ThingsBoard node ...");
              if ( client.connect("Esp32COSensor", TOKEN_TB, NULL) ) {
                 Serial.println( "[DONE]" );
                 int rsus=client.subscribe("v1/devices/me/rpc/request/+");
                 Serial.print( "[SUBSCRIBE]" );
                 Serial.println(rsus);
                 rsus=client.publish("v1/devices/me/attributes", get_gpio_status().c_str()); 
                 Serial.print("Enviando GPIO status ...");
                 Serial.println(rsus);
              } else {
                 Serial.print( "[FAILED] [ rc = " );
                 Serial.print( client.state() );
              }
          }
          //Serial.println("Client LOOP");
          //Serial.print("LOOP ");
          if(! client.loop()){
            Serial.println("Error en Loop.");  
          }
          //Serial.println(taskMessage);
      }else{
      Serial.print("No conectado wifi:");
      Serial.println(Wconectado);
    }
   delay(1000);
   }
}

void setup() { 
  Serial.begin(115200);  
  pinMode(21,INPUT_PULLUP);
  pinMode(22,INPUT_PULLUP);             
  pinMode(RLY_I1, OUTPUT);
  pinMode(RLY_I2, OUTPUT);
  pinMode(RLY_I3, OUTPUT);
  pinMode(RLY_I4, OUTPUT);
  pinMode(DHT_PWR, OUTPUT);
  pinMode(DHT_DATA,INPUT_PULLUP);
  digitalWrite(DHT_PWR, HIGH);
  analogReadResolution(12); //12 bits
  analogSetAttenuation(ADC_11db);  //For all pins
  analogSetPinAttenuation(ANALOG_PIN_0, ADC_11db); //0db attenu
  delay(400);
  dht.begin();
  mq2.begin(); 
  client.setServer(mqtt_server, 1883);
  client.setCallback(on_message);
  
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  }
    
  xTaskCreatePinnedToCore(coreTask, "coreTask", 10000, NULL, 0, NULL, taskCore);
  Serial.println("Hilo Creado...");  
}


void loop() {
 if (Wconectado == 0){
   Serial.println("Error No conectado wifi Wifi_init.");
   Wifi_init();
 }else{
    
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Error obteniendo los datos del sensor DHT11");
    delay(600);
    return;
  }
  float hic = dht.computeHeatIndex(t, h, false);
  
  float* values= mq2.read(true); //set it false if you don't want to print the values in the Serial
  //Reading specific values:
  //lpg = values[0];
  lpg = mq2.readLPG();
  //co = values[1];
  co = mq2.readCO();
  //smoke = values[2];
  smoke = mq2.readSmoke();
  Serial.print("Valor CO:");
  Serial.println(co);
  Serial.print("Humedad :");
  Serial.print(h);
  Serial.print(" Temperatura :");
  Serial.print(t);
  Serial.print(" Sensacion Termica :");
  Serial.println(hic);

  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
    
  Serial.print("Pressure = ");
  pres=bmp.readPressure();
  Serial.print(pres);
  Serial.println(" Pa");
  
  String taskMessage = "Corriendo en core ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);
  send_mqtt(t,h,hic,co,lpg,smoke,pres);
 }
 delay(60000); 
}
 

