#include <WiFi.h>

const char* ssid="myGate";
const char* password="192837465";

const uint16_t port= 5555;
const char * host ="192.168.137.1";//"192.168.0.207";

void setup() {
  
   Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");
  }
 
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());

}

void loop() {
  
   WiFiClient client;
 
    if (!client.connect(host, port)) {
 
        Serial.println("Connection to host failed");
 
        delay(1000);
        return;
    }
 
    Serial.println("Connected to server successful!");
 
    
    
    
    client.print("");
 
}
