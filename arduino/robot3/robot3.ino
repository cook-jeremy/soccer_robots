#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>

// ROBOT 3

// Static IP details...
IPAddress ip(192, 168, 1, 203);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(0, 0, 0, 0);

#define ssid "MontyPylon"
#define pass "soccer_robots"
WiFiUDP Udp;
unsigned int localPort = 2390; // local port to listen on
char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged"; // a string to send back

#define LED 5
Adafruit_NeoPixel strip = Adafruit_NeoPixel(5, LED, NEO_GRB + NEO_KHZ800);
uint32_t red = strip.Color(255, 0, 0);
uint32_t green = strip.Color(0, 255, 0);
uint32_t blue = strip.Color(0, 0, 255);
uint32_t none = strip.Color(0, 0, 0);

int rightB = 12;
int rightF = 15;
int leftF = 14;
int leftB = 4;

void setup() {
  pinMode(leftF, OUTPUT);
  pinMode(leftB, OUTPUT);
  pinMode(rightF, OUTPUT);
  pinMode(rightB, OUTPUT);
  analogWrite(rightF, 0);
  analogWrite(leftF, 0);
  analogWrite(rightB, 0);
  analogWrite(leftB, 0);

  //LED
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(64);
  
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  setColor("red");
  strip.show();

  
  // Static IP Setup Info Here...
  WiFi.config(ip, gateway, subnet, dns);

  WiFi.begin(ssid, pass);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to wifi");
  printWifiStatus();

  setColor("green");
  delay(500);
  setColor("none");

  Serial.println("\nWaiting for incoming messages...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
}

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    //Serial.print("Received packet of size ");
    //Serial.println(packetSize);
    //Serial.print("From ");
    //IPAddress remoteIp = Udp.remoteIP();
    //Serial.print(remoteIp);
    //Serial.print(", port ");
    //Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    String leftString = "aaaaa";
    leftString.setCharAt(0, packetBuffer[0]);
    leftString.setCharAt(1, packetBuffer[1]);
    leftString.setCharAt(2, packetBuffer[2]);
    leftString.setCharAt(3, packetBuffer[3]);
    leftString.setCharAt(4, packetBuffer[4]);
    int leftPower = leftString.toInt();

     String rightString = "aaaaa";
    rightString.setCharAt(0, packetBuffer[5]);
    rightString.setCharAt(1, packetBuffer[6]);
    rightString.setCharAt(2, packetBuffer[7]);
    rightString.setCharAt(3, packetBuffer[8]);
    rightString.setCharAt(4, packetBuffer[9]);
    int rightPower = rightString.toInt();

    Serial.print("Left power: ");
    Serial.println(leftPower);
    Serial.print("Right power: ");
    Serial.println(rightPower);

    
    if(leftPower > 0 && leftPower <=1023) {
      analogWrite(leftB, 0);
      analogWrite(leftF, leftPower);
    } else if(leftPower == 0) {
      analogWrite(leftF, 0);
      analogWrite(leftB, 0);
    } else if(leftPower < 0 && leftPower >= -1023) {
      analogWrite(leftF, 0);
      int posLeftPower = abs(leftPower);
      analogWrite(leftB, posLeftPower);
    }

    if(rightPower > 0 && rightPower <=1023) {
      analogWrite(rightB, 0);
      analogWrite(rightF, rightPower);
    } else if(rightPower == 0) {
      analogWrite(rightF, 0);
      analogWrite(rightB, 0);
    } else if(rightPower < 0 && rightPower >= -1023) {
      analogWrite(rightF, 0);
      int posRightPower = abs(rightPower);
      analogWrite(rightB, posRightPower);
    }
    
    
    // send a reply, to the IP address and port that sent us the packet we received
    //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    //Udp.write(ReplyBuffer);
    //Udp.endPacket();
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void setColor(String color) {
  if(color == "red") {
    for(int a = 0; a < 5; a++) {
      strip.setPixelColor(a, red);
    }
    strip.show();
  } else if(color == "green") {
    for(int a = 0; a < 5; a++) {
      strip.setPixelColor(a, green);
    }
    strip.show();
  } else if(color == "blue") {
    for(int a = 0; a < 5; a++) {
      strip.setPixelColor(a, blue);
    }
    strip.show();
  } else if(color == "none") {
    for(int a = 0; a < 5; a++) {
      strip.setPixelColor(a, none);
    }
    strip.show();
  }
}
