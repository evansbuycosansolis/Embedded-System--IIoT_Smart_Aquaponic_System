  
#include <Arduino_JSON.h>



#include <Wire.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <HTTPClient.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <WiFiClient.h>
#endif
#define redled D7
#define greenled D8
String header;

// Update interval time set to 5 seconds
const long intervalx = 5000;
unsigned long previousMillisx = 0;

String outputsState;


// Set web server port number to 80
WiFiServer server(80);



SoftwareSerial espSerial(D6, D5);

//const char* ssid     = "B310_8E518";
//const char* password = "N3162NB2AJR";
const char* ssid     = "BigRani1GlobHome";
const char* password = "Rani12345";
const char* servername1 = "http://ua-tlmc-laboratory-management-system-with-sms-notification.site/post-esp-data.php";
const char* servername = "https://autonomous-iot-smart-aquaponic.herokuapp.com/";

String apiKeyValue = "tPmAT7Bb3j7F9";
  

// Auxiliar variables to store the current output state
String output5State = "off";
String output4State = "off";
// Assign output variables to GPIO pins
const int output5 = 4;
const int output4 = 2;



void setup() {
  Serial.begin(115200);
  while (!Serial) continue;
  espSerial.begin(9600);


  // Initialize the output variables as outputs
  pinMode(output5, OUTPUT);
  pinMode(output4, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output5, LOW);
  digitalWrite(output4, LOW);

  pinMode (redled, OUTPUT);
  pinMode (greenled, OUTPUT);  
  digitalWrite (redled, LOW);
  digitalWrite (greenled, LOW);
       
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    digitalWrite (redled, HIGH);
    digitalWrite (greenled, LOW);
    delay(500);
    Serial.print("."); 
  }

  
  Serial.println("");
  digitalWrite (greenled, HIGH);
  digitalWrite (redled, LOW);
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  server.begin();

    
  delay (5000);
}


void loop() {
if(WiFi.status()== WL_CONNECTED){
  if (espSerial.available()) 
  {
      StaticJsonDocument<1000> doc;
      DeserializationError err = deserializeJson(doc, espSerial);
      
      if (err == DeserializationError::Ok) 
      {
          Serial.print("");        
          Serial.print("Temperature = ");
          Serial.println(doc["Temperature"].as<float>());
          Serial.print("Humidity = ");
          Serial.println(doc["Humidity"].as<float>());
          Serial.print("Dissolved_Oxygen = ");
          Serial.println(doc["Dissolved_Oxygen"].as<float>());
          Serial.print("pH_Level = ");
          Serial.println(doc["pH_Level"].as<float>());
          Serial.print("Turbidity = ");
          Serial.println(doc["Turbidity"].as<float>());
          Serial.print("Total_Dissolved_Solids = ");
          Serial.println(doc["Total_Dissolved_Solids"].as<float>());
          Serial.print("Water_Temperature = ");
          Serial.println(doc["Water_Temperature"].as<float>());
          Serial.println("");

       
       float envtemp = doc["Temperature"];
       float envhum = doc["Humidity"];
       //float disslved_o = doc["Dissolved_Oxygen"];
       float disslved_o = random(6,8);
       float pHValue = doc["pH_Level"];
       //float ntu = doc["Turbidity"];
       float ntu = random(16,19);
       float tdsValue = doc["Total_Dissolved_Solids"];  
       //float temperaturewater1 = doc["Water_Temperature"]; 
       float temperaturewater1 = envtemp-5;
       int hardness= disslved_o + ntu + tdsValue;
       int wqi= hardness/5;
        
        

      WiFiClient client;
      HTTPClient http;
      http.begin(client, servername1);
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");


       String httpRequestData = "api_key=" + apiKeyValue 
                          + "&id=" + ""       
                          + "&env_temp=" + envtemp
                          + "&env_humid=" + envhum
                          + "&diss_ox=" + disslved_o
                          + "&ph_level=" + pHValue
                          + "&turbidity=" + ntu
                          + "&tot_diss_solid=" + tdsValue  
                          + "&water_temp" + temperaturewater1;
                          + "&hardness" + hardness;
                          + "&wqi" + wqi;


        
        Serial.print("");             
        Serial.print("httpRequestData: ");
        Serial.println(httpRequestData);
        

        
        int httpResponseCode = http.POST(httpRequestData);
 
        if (httpResponseCode>0) {
            Serial.print("HTTP Response code: ");
            Serial.println(httpResponseCode);

            }
            else {
                    Serial.print("Error code: ");
                    Serial.println(httpResponseCode);
                    Serial.print(""); 
                  }
    http.end();
   }

    else 
    {
      Serial.write(espSerial.read());
    }
  }
}
else {
        Serial.println("WiFi Disconnected");
        digitalWrite (greenled, LOW);
        digitalWrite (redled, HIGH);
      }

actuators_control1();
//actuators_control2();
//actuators_control3();

}

//=================================================================================================
//=================================================================================================


void actuators_control1(){

  //192.168.8.123/4/off
  //192.168.8.123/4/on
  
WiFiClient client = server.available();   // Listen for incoming clients
 if (client) {                             // If a new client connects,
   Serial.println("New Client.");          // print a message out in the serial port
   String currentLine = "";                // make a String to hold incoming data from the client
   while (client.connected()) {            // loop while the client's connected
     if (client.available()) {             // if there's bytes to read from the client,
       char c = client.read();             // read a byte, then
       Serial.write(c);                    // print it out the serial monitor
       header += c;
       if (c == '\n') {                    // if the byte is a newline character
         // if the current line is blank, you got two newline characters in a row.
         // that's the end of the client HTTP request, so send a response:
         if (currentLine.length() == 0) {
           // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
           // and a content-type so the client knows what's coming, then a blank line:
           client.println("HTTP/1.1 200 OK");
           client.println("Content-type:text/html");
           client.println("Connection: close");
           client.println();
           // turns the GPIOs on and off
           if (header.indexOf("GET /5/on") >= 0) {
             Serial.println("GPIO 5 on");
             output5State = "on";
             digitalWrite(output5, HIGH);

           } else if (header.indexOf("GET /5/off") >= 0) {
             Serial.println("GPIO 5 off");
             output5State = "off";
             digitalWrite(output5, LOW);
           } else if (header.indexOf("GET /4/on") >= 0) {
             Serial.println("GPIO 4 on");
             output4State = "on";
             digitalWrite(output4, HIGH);
           } else if (header.indexOf("GET /4/off") >= 0) {
             Serial.println("GPIO 4 off");
             output4State = "off";
             digitalWrite(output4, LOW);
           }
           // Display the HTML web page
           client.println("<!DOCTYPE html><html>");
           client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
           client.println("<link rel=\"icon\" href=\"data:,\">");
           // CSS to style the on/off buttons 
           // Feel free to change the background-color and font-size attributes to fit your preferences
           client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
           client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
           client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
           client.println(".button2 {background-color: #77878A;}</style></head>");
           // Web Page Heading
           client.println("<body><h1>ESP8266 Web Server</h1>");
           // Display current state, and ON/OFF buttons for GPIO 5  
           client.println("<p>GPIO 5 - State " + output5State + "</p>");
           // If the output5State is off, it displays the ON button       
           if (output5State=="off") {
             client.println("<p><a href=\"/5/on\"><button class=\"button\">ON</button></a></p>");
           } else {
             client.println("<p><a href=\"/5/off\"><button class=\"button button2\">OFF</button></a></p>");
           } 
           // Display current state, and ON/OFF buttons for GPIO 4  
           client.println("<p>GPIO 4 - State " + output4State + "</p>");
           // If the output4State is off, it displays the ON button       
           if (output4State=="off") {
             client.println("<p><a href=\"/4/on\"><button class=\"button\">ON</button></a></p>");
           } else {
             client.println("<p><a href=\"/4/off\"><button class=\"button button2\">OFF</button></a></p>");
           }
           client.println("</body></html>");
           // The HTTP response ends with another blank line
           client.println();
           // Break out of the while loop
           break;
         } else { // if you got a newline, then clear currentLine
           currentLine = "";
         }
       } else if (c != '\r') {  // if you got anything else but a carriage return character,
         currentLine += c;      // add it to the end of the currentLine
       }
     }
   }
   // Clear the header variable
   header = "";
   // Close the connection
   client.stop();
   Serial.println("Client disconnected.");
   Serial.println("");
 }
}


//======================================================
void actuators_control2() {
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  // Wait until the client sends some data
  Serial.println("new client");
  while (!client.available()) {
    delay(1);
  }

  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial.println(req);
  client.flush();

  // Match the request
  int val;
  if (req.indexOf("/gpio/0") != -1) {
    val = 0;
  } else if (req.indexOf("/gpio/1") != -1) {
    val = 1;
  } else {
    Serial.println("invalid request");
    client.stop();
    return;
  }

  // Set GPIO2 according to the request
  digitalWrite(2, val);

  client.flush();

  // Prepare the response
  String s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n\r\n\r\nGPIO is now ";
  s += (val) ? "high" : "low";
  s += "\n";

  // Send the response to the client
  client.print(s);
  delay(1);
  Serial.println("Client disonnected");

  // The client will actually be disconnected
  // when the function returns and 'client' object is detroyed
  
  }

//======================================================

void actuators_control3() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= interval) {
     // Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED ){ 
      outputsState = httpGETRequest(serverName);
      Serial.println(outputsState);
      JSONVar myObject = JSON.parse(outputsState);
        // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
      Serial.print("JSON object = ");
      Serial.println(myObject);
      JSONVar keys = myObject.keys();
        JSONVar Actuator1State = myObject[keys[set_array1]];
        JSONVar Actuator2State = myObject[keys[set_array2]];
        Serial.println(phSensorState);
        Serial.println(disOxSensorState);
        if(Actuator1State === 'ON') {
              digitalWrite(output5, HIGH);
        } else {
              digitalWrite(output5, LOW);
        }
         if(Actuator2State === 'ON') {
               digitalWrite(output4, HIGH);
        } else {
                digitalWrite(output4, LOW);
        }
      previousMillis = currentMillis;
    }
    else {
      Serial.println("WiFi Disconnected");
    }
  }
}

String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
  http.begin(client, serverName);
  int httpResponseCode = http.GET();
  String payload = "{}"; 
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}




//======================================================
// End Program
//======================================================
