#include <dht11.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

#define airLedPin D6
#define connLedPin D7
WiFiServer server(80);

String header;
int rainCheck = 0;
double airQuality = 0;

dht11 DHT11;
Adafruit_BMP280 bmp;

void setup()
{
  Serial.begin(9600);
  WiFi.begin("Vladi", "davidcrofts");
  Serial.println();
  Serial.println("hello!");
  Serial.println("sensor test");
  while (!bmp.begin(0x76))
  {
    Serial.print("*");
    delay(500);
  }
  Serial.println();
  Serial.println("detected bmp280!");

  Serial.println("connecting to inet");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("*");
    delay(500);
  }
  Serial.println();
  Serial.print("connected to ");
  Serial.println(WiFi.localIP());
  server.begin();

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,    
                  Adafruit_BMP280::SAMPLING_X2,    
                  Adafruit_BMP280::SAMPLING_X16,   
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 

  pinMode(airLedPin, OUTPUT);
  pinMode(connLedPin, OUTPUT);
}

void loop()
{
  if (millis() % 9680 == 0)
  {
    digitalWrite(airLedPin, LOW);
    delayMicroseconds(280);
    double aMeasure = analogRead(A0);
    delayMicroseconds(40);
    digitalWrite(airLedPin, HIGH);

    double calcVoltage = aMeasure * (3.3/ 1024.0);
    airQuality = 170 * calcVoltage - 0.1;
  }
  WiFiClient client = server.available(); //>:(
  if (client)
  {
    digitalWrite(connLedPin, HIGH);
    Serial.println();
    Serial.println("client connected");
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();
    
    //html definition
    client.println("<!DOCTYPE html><html>");
    client.println("<head><meta http-equiv=\"refresh\" content=\"6\"></head>");
    client.println("<body><h1>Humidity and temp</h1>");

  int dat = DHT11.read(D3);
  if (digitalRead(D0) == HIGH)
  {
    rainCheck = 1;
  }
  else
  {
    rainCheck = 0;
  }
    
    //measuring part
    client.print("<p>Humidity = ");
    client.print((double)DHT11.humidity, 2);
    client.println(" %</p>");
    client.print("<p>Temperature = ");
    client.print(bmp.readTemperature() - 2);
    client.println(" degrees C</p>");
    client.print("<p>Rain? = ");
    client.print(rainCheck);
    client.println("</p>");
    client.print("<p>Pressure = ");
    client.print(bmp.readPressure());
    client.println(" Pa</p>");
    client.print("<p>Air quality: ");
    client.print(airQuality);
    client.println(" ug/m3</p>");

    client.println("</body></html>");
  }
  
  else
  {
    digitalWrite(connLedPin, LOW);
  }
  client.stop();
}
