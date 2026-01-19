#include <micro_ros_arduino.h>
#include <WiFi.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

// ================= WIFI =================
#define WIFI_SSID     "Airtel_yoge_5897"
#define WIFI_PASSWORD "air52842"

#define AGENT_IP   "192.168.1.6"   // PC IP
#define AGENT_PORT 8888

// ================= MOTOR PINS =================
// EXACT SAME PINS YOU TESTED
#define IN1 26
#define IN2 27
#define IN3 33
#define IN4 13

// ================= micro-ROS =================
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ================= SAFETY =================
unsigned long last_cmd_time = 0;
#define CMD_TIMEOUT_MS 500   // stop if no cmd_vel in 500 ms

// ================= MOTOR FUNCTIONS =================
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// ================= CALLBACK =================
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * twist =
    (const geometry_msgs__msg__Twist *)msgin;

  float linear  = twist->linear.x;
  float angular = twist->angular.z;

  last_cmd_time = millis();  // heartbeat

  Serial.print("linear.x = ");
  Serial.print(linear);
  Serial.print("  angular.z = ");
  Serial.println(angular);

  if (linear > 0.05) forward();
  else if (linear < -0.05) backward();
  else if (angular > 0.05) left();
  else if (angular < -0.05) right();
  else stopMotors();
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  // Disable WiFi sleep (VERY IMPORTANT)
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // micro-ROS transport
  set_microros_wifi_transports(
    WIFI_SSID,
    WIFI_PASSWORD,
    AGENT_IP,
    AGENT_PORT
  );

  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_motor_node", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &cmd_vel_callback,
    ON_NEW_DATA
  );

  Serial.println("micro-ROS motor node started");
}

// ================= LOOP =================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // SAFETY STOP if cmd_vel stops
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }
}
