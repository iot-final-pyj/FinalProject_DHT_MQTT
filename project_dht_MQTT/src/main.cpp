#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include "DHT.h" // DHT 센서를 위한 라이브러리

// 핀 설정
#define DHTPIN 15         // DHT22 데이터 핀
#define DHTTYPE DHT22     // DHT22 센서 타입
#define FAN1_PWM_PIN 16   // 팬1 PWM 핀
#define FAN2_PWM_PIN 17   // 팬2 PWM 핀

// PWM 설정
const int pwmChannel1 = 0;        // 팬1 PWM 채널
const int pwmChannel2 = 1;        // 팬2 PWM 채널
const int pwmFrequency = 25000;  // PWM 주파수 (25kHz)
const int pwmResolution = 8;     // PWM 해상도 (8비트: 0~255)

// 임계값 설정
const float HIGH_TEMP = 25.5;      // 고온 기준
const float MID_TEMP = 25.0;       // 중간 온도 기준
const float HIGH_HUMIDITY = 50.0;  // 높은 습도 기준
const float MID_HUMIDITY = 40.0;   // 중간 습도 기준

// WiFi 설정
const char* ssid = "IoT518";          // WiFi SSID
const char* password = "iot123456";   // WiFi Password

// MQTT 브로커 설정
const char* mqtt_server = "192.168.0.249"; // Mosquitto 브로커 IP
const int mqtt_port = 1883;
const char* mqtt_topic = "home/temperature"; // MQTT 주제(topic)

// DHT 센서 객체 생성
DHT dht(DHTPIN, DHTTYPE);

// WiFi 및 MQTT 클라이언트 객체 생성
WiFiClient espClient;
PubSubClient client(espClient);

// WiFi 연결 함수
void setup_wifi() {
  delay(10);
  Serial.println("WiFi 연결 중...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi 연결 성공!");
  Serial.print("IP 주소: ");
  Serial.println(WiFi.localIP());
}

// MQTT 재연결 함수
void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT 브로커에 연결 중...");
    if (client.connect("ESP32Client")) {
      Serial.println("연결 성공!");
    } else {
      Serial.print("연결 실패, 상태 코드=");
      Serial.print(client.state());
      Serial.println(" 다시 시도 중...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);

  // PWM 핀 설정
  ledcSetup(pwmChannel1, pwmFrequency, pwmResolution);
  ledcAttachPin(FAN1_PWM_PIN, pwmChannel1);

  ledcSetup(pwmChannel2, pwmFrequency, pwmResolution);
  ledcAttachPin(FAN2_PWM_PIN, pwmChannel2);

  Serial.println("DHT22 Fan Control and MQTT Publisher Initialized");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // DHT22 데이터 읽기
  float temperature = dht.readTemperature(); // 섭씨 온도
  float humidity = dht.readHumidity();       // 습도

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("DHT22 데이터 읽기 실패!");
    delay(2000);
    return;
  }

  // 온도 및 습도 출력
  Serial.printf("Temperature: %.2f °C, Humidity: %.2f %%\n", temperature, humidity);

  // 팬 제어 조건
  if (temperature >= HIGH_TEMP || humidity >= HIGH_HUMIDITY) {
    // 온도나 습도가 매우 높은 경우: 2개의 팬 모두 최대 속도로 작동
    Serial.println("Both fans running at full speed");
    ledcWrite(pwmChannel1, 255); // Fan 1: Full Speed
    ledcWrite(pwmChannel2, 255); // Fan 2: Full Speed
  } else if (temperature >= MID_TEMP || humidity >= MID_HUMIDITY) {
    // 온도나 습도가 약간 높은 경우: 1개의 팬만 작동
    Serial.println("Fan 1 running at medium speed");
    ledcWrite(pwmChannel1, 128); // Fan 1: Medium Speed
    ledcWrite(pwmChannel2, 0);   // Fan 2: Off
  } else {
    // 온도와 습도가 모두 낮은 경우: 팬 모두 중지
    Serial.println("Fans stopped");
    ledcWrite(pwmChannel1, 0);   // Fan 1: Off
    ledcWrite(pwmChannel2, 0);   // Fan 2: Off
  }

  // MQTT로 온도 및 습도 데이터 전송
  String payload = String("{\"temperature\":") + temperature + ",\"humidity\":" + humidity + "}";
  client.publish(mqtt_topic, payload.c_str());
  Serial.println("MQTT 메시지 전송: " + payload);

  delay(5000); // 5초마다 데이터 전송
}
