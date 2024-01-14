#include <SPI.h>
#include <cmath>
#include <Adafruit_MPU6050.h>  // emu
#include <Chrono.h>
#include <math.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

//#define D8 15 //CSB
#define D7 13  //MOSI
//#define D6 13 //SDA, MISO
//#define D5 14 //clock, SCL

Adafruit_MPU6050 mpu;

//Adafruit_BMP280 bmp(D8, D6, D7, D5);

Chrono LoopTimer, FlightTimer, ActualFlightTimer;
int dt = 1;

IPAddress staticIP(192, 168, 1, 4); // Set the static IP address
IPAddress gateway(192, 168, 1, 100); // Set your gateway IP address
IPAddress subnet(255, 255, 255, 0); // Set your subnet mask


//circular thing
class CircularBuffer {
private:
  std::vector<float> buf_;
  size_t head_ = 0;
  size_t tail_ = 0;
  const size_t max_size_;
  bool full_ = false;

public:
  CircularBuffer(size_t size)
    : max_size_(size), buf_(std::vector<float>(size)) {}

  // std::vector<float> buf_;
  float last() {  // freshest one
    // The 'head_' points to the next insertion point, so the last element is just before it
    return buf_[(head_ + max_size_ - 1) % max_size_];
  }
  float first() {
    return buf_[tail_];  //oldest one
  }
  void put(float item) {
    buf_[head_] = item;
    head_ = (head_ + 1) % max_size_;
    if (full_) {
      tail_ = (tail_ + 1) % max_size_;
    }
    full_ = head_ == tail_;
  }

  float get() {
    if (empty()) {
      return 0.0f;  // Return a default value if the buffer is empty
    }
    float val = buf_[tail_];
    full_ = false;
    tail_ = (tail_ + 1) % max_size_;
    return val;
  }

  bool empty() const {
    return (!full_ && (head_ == tail_));
  }

  size_t size() const {
    if (full_) {
      return max_size_;
    }
    if (head_ >= tail_) {
      return head_ - tail_;
    }
    return max_size_ + head_ - tail_;
  }
  float average() {
    float sum = 0;
    for (size_t i = 0; i < this->size(); i++) {
      sum += buf_.at(i);
    }
    return sum / this->size();
  }
};

//
//
//
static CircularBuffer buffer_a(3);
static CircularBuffer buffer_accx(10);
static CircularBuffer buffer_b(3);
static CircularBuffer buffer_c(5);
float calx, caly, calz;
static float maxAltitude = 0;
static int flag_remote_arm = 0;
static int flag_launched = 0;
static int flag_armed = 0;
static int flag_deployed = 0;
float loop_time = 0;
static float avg =0, g = 0;
static float roll = 0.0; //x
static float pitch = 0.0; //y
static float yaw = 0.0;  //z
static float deltav = 0.0; 
static long int prev_time = 0.0; 
float G = 10;
float gx = 10;
//
//
//
const char *ssid = "PENTAGRAM";
const char *password = "12345678";  // Password can be between 8 and 64 characters
ESP8266WebServer server(80);
void handleRoot() {
  
  String HTML = "<html><head><meta http-equiv=\"refresh\" content=\"1\"><title>ESP8266 Parachute Deploy</title><style>"
                "body{background-color:#def;font-family:Arial,sans-serif;text-align:center;margin-top:50px;}.button{padding:15px 25px;font-size:24px;text-align:center;cursor:pointer;outline:none;color:#fff;background-color:#008CBA;border:none;border-radius:15px;box-shadow:0 9px #999;}.button:hover{background-color:#0077A7}.button:active{background-color:#0077A7;box-shadow:0 5px #666;transform:translateY(4px);}"
                "</style></head><body><h1>ESP8266 Parachute Controller</h1>"
                "<p>Actual Flight Time: " + String(ActualFlightTimer.elapsed() / 1000) + "</p>"
                "<p>Avionics Runtime: " + String(FlightTimer.elapsed() / 1000) + "</p>"
                "<p>Flag Remote Arm: " + String(flag_remote_arm) + "</p>"
                "<p>Flag Launched: " + String(flag_launched) + "</p>"
                "<p>Flag Armed: " + String(flag_armed) + "</p>"
                "<p>Flag Deployed: " + String(flag_deployed) + "</p>"
                "<p>Acceleration: " + String(g) + "</p>"
                "<p>Avg Acceleration: " + String(avg) + "</p>"
                "<p>Prograde Velocity: " + String(deltav) + "</p>"
                "<p>Signal Strength: " + String(WiFi.RSSI()) + " dBm</p>"
                "<p>Router IP: 192.168.100." + String(WiFi.gatewayIP()[3]) + "</p>"
                "<form action=\"/deploy\"><input type=\"submit\" class=\"button\" value=\"DEPLOY PARACHUTE!\"/></form>"
                "<form action=\"/launch\"><input type=\"submit\" class=\"button\" value=\"Set Launch Flag\"/></form>"
                "<form action=\"/arm\"><input type=\"submit\" class=\"button\" value=\"Arm Rocket\"/></form>"
                "</body></html>";

  server.send(200, "text/html", HTML);
}




void handleDeployParachute() {
  digitalWrite(D7, HIGH);  // Set TX GPIO 1 high
  delay(1000);
  digitalWrite(D7, LOW);
  server.send(200, "text/plain", "Parachute Deployed!");
}

void handleArmRocket() {
  flag_remote_arm = 1;
  delay(1000);
  server.send(200, "text/plain", "Rocket Armed!");
}


void deployParachute() {
  digitalWrite(D7, HIGH);  // Set TX GPIO 1 high
  flag_deployed = 1;
  delay(1000);
  digitalWrite(D7, LOW);
}

//
void handleLaunch() {
  flag_launched = 1;
  server.send(200, "text/plain", "Launch flag set to true!");
}



struct Wekt {
  float x, y, z;
};

float vectorLength(float x, float y, float z) {
  return sqrt((x * x) + (y * y) + (z * z));
}
///
//SETUP
///
void setup() {
  FlightTimer.start();
  ActualFlightTimer.stop();
  Serial.begin(9600);
  //bmp.begin();

  //makes sure the circular buffer is like always there for you
  pinMode(D7, OUTPUT);    // Initialize parachute fire pin
  digitalWrite(D7, LOW);  // Set low

  if (!mpu.begin()) { Serial.println("MPU fail"); }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);
  //calibrate gyro


  calx = gyro.gyro.x;
  caly = gyro.gyro.y;
  calz = gyro.gyro.z;
  delay(1000);
  mpu.getEvent(&acc, &gyro, &temp);
  calx = 0.5 * (calx + gyro.gyro.x);
  caly = 0.5 * (caly + gyro.gyro.y);
  calz = 0.5 * (calx + gyro.gyro.z);

  
  WiFi.begin(ssid, password);
  
  WiFi.config(staticIP, gateway, subnet);


  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  IPAddress IP = WiFi.localIP();
  Serial.print("Connected to WiFi with IP address: ");
  Serial.println(IP);
  //
  //
  // Setting the ESP as an access point
 // Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter if you want the AP to be open
  //WiFi.softAP(ssid, password);

  //IPAddress IP = WiFi.softAPIP();                         // Get the ESP's IP address
  server.on("/", HTTP_GET, handleRoot);                   // Serve the HTML form at root
  server.on("/deploy", HTTP_GET, handleDeployParachute);  // Handle form submission
  server.on("/launch", HTTP_GET, handleLaunch);           // Handle setting launch flag
  server.on("/arm", HTTP_GET, handleArmRocket);  // Handle arming the rocket

  server.begin();
  Serial.println("HTTP server started");
  //
}
void loop() {
  //timers
  
  long int time = millis() / 1000;
  
  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);
  //Serial.println("STOP--");
  float fcalx = -0.0005;
  float fcaly = -0.001;
  float fcalz = -0.038770;

  float accX = acc.acceleration.x;
  float accY = acc.acceleration.y;
  float accZ = acc.acceleration.z;

  float noisyrotX = gyro.gyro.x; //- calx + fcalx;
  float noisyrotY = gyro.gyro.y; //- caly + fcaly;
  float noisyrotZ = gyro.gyro.z; //- calz + fcalz;

  float rotX = (100*noisyrotX)/100;
  float rotY = (100*noisyrotY)/100;
  float rotZ = (100*noisyrotZ)/100;

  dt = LoopTimer.elapsed();
  LoopTimer.start();

  roll += (rotX * dt/1000)*180/3.14; // integrate X-axis rotation rate to get roll angle
  pitch += (rotY * dt/1000)*180/3.14; // integrate Y-axis rotation rate to get pitch angle
  yaw += (rotZ * dt/1000)*180/3.14; // integrate Z-axis rotation rate to get yaw angle
    buffer_accx.put(accX);
    gx = buffer_accx.average();
  if(prev_time!=time){
      deltav += std::floor(accX);
  }
  prev_time=time;
  Serial.println(deltav);
  
  g = vectorLength(accX, accY, accZ);
  buffer_a.put(g);
  Serial.println(" ");

  avg = buffer_a.average();


  //!!!!!!!! Launched flag
  delay(25);

  if (flag_remote_arm &&(avg > 25.0) && (FlightTimer.elapsed() > 1000)) {
    flag_launched = 1;
    ActualFlightTimer.start();
  }

  if (flag_launched && !flag_armed && (avg < 1) && (ActualFlightTimer.elapsed() > 3000)) {
    flag_armed = 1;
  }

  if (!flag_deployed && flag_launched && flag_armed && (avg > 5) && (ActualFlightTimer.elapsed() > 4500)) {
    flag_deployed = 1;
    deployParachute();
  }
  if (flag_remote_arm &&flag_launched && (ActualFlightTimer.elapsed() > 8000)) {
    flag_deployed = 1;
    deployParachute();
  }

  float first = buffer_a.first();
  float last = buffer_a.last();
  /*
  if (flag_armed && (avg > 10)) {
    flag_deployed = 1;  //
    deployParachute();
  };
  */
  
  //
  server.handleClient();
  //
  /*
  Serial.println();
  Serial.printf("g: %f, avg: %f, launched: %d, armed: %d, deployed: %d", g, avg, flag_launched, flag_armed, flag_deployed);
  Serial.println("\n");
  */
  //Serial.printf("%f, %f, %f, %d \n",gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, dt, FlightTimer.elapsed());
  //Serial.printf("g: %f, avg: %f, launched: %d, armed: %d, deployed: %d", g, avg, flag_launched, flag_armed, flag_deployed);
  //Serial.printf("%f, %f, %f, %d \n", rotX, rotY, rotZ, dt, FlightTimer.elapsed());
  //Serial.printf("%f, %f, %f, %d \n", roll, pitch, yaw, dt, FlightTimer.elapsed());
  //Serial.println(WiFi.softAPgetStationRSSI(i));
  //rad/s

  //Serial.println(temp.temperature);
  
  


  //transmisja danych - rakieta Alicja
}
