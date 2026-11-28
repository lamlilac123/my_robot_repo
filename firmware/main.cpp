#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/ledc.h"
#include "driver/gpio.h"

// Khai báo chân kết nối
#define PWMA GPIO_NUM_21
#define AIN2 GPIO_NUM_19
#define AIN1 GPIO_NUM_18
#define STBY GPIO_NUM_5
#define BIN1 GPIO_NUM_17
#define BIN2 GPIO_NUM_16
#define PWMB GPIO_NUM_4

#define PAN GPIO_NUM_25
#define TILT GPIO_NUM_26

// /dev/ttyUSB0
// kiến thức mới
const char *WIFI_SSID = "CEEC_Tenda";
const char *WIFI_PASS = "1denmuoi1";
const uint16_t UDP_PORT = 4210;
WiFiUDP udp;
uint8_t rx_buf[256];
size_t rx_len = 0;
size_t rx_idx = 0;

const uint32_t MAX_DUTY_MOTOR = 255;
const uint32_t MAX_DUTY_SERVO = 16383;

uint8_t pan_angle = 90;
uint8_t tilt_angle = 90;
float speed_limit = 0.3;
void setup_gpio(gpio_num_t out_gpio)
{
  const gpio_config_t io_conf_out = {
      .pin_bit_mask = (1ULL << out_gpio), // GPIO
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE // INTERUPT
  };
  gpio_config(&io_conf_out);
}

void setup_pwm_motor(gpio_num_t pwm_A, gpio_num_t pwm_B)
{
  const ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_8_BIT, // RESOLUTION 0-255
      .timer_num = LEDC_TIMER_0,           // TIMER
      .freq_hz = 20000,                    // Hz
      .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&ledc_timer);

  const ledc_channel_config_t ledc_channel_A = {
      .gpio_num = pwm_A, // GPIO
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,      // CHANNEL
      .intr_type = LEDC_INTR_DISABLE, // INTERUPT
      .timer_sel = LEDC_TIMER_0,      // TIMER
      .duty = 0,                      // OVERFLOW VALUE
      .hpoint = 0};
  ledc_channel_config(&ledc_channel_A);

  const ledc_channel_config_t ledc_channel_B = {
      .gpio_num = pwm_B, // GPIO
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_1,      // CHANNEL
      .intr_type = LEDC_INTR_DISABLE, // INTERUPT
      .timer_sel = LEDC_TIMER_0,      // TIMER
      .duty = 0,                      // OVERFLOW VALUE
      .hpoint = 0};
  ledc_channel_config(&ledc_channel_B);
}

void setup_pwm_servo(gpio_num_t pwm_PAN, gpio_num_t pwm_TILT)
{
  const ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_14_BIT, // RESOLUTION max=16385
      .timer_num = LEDC_TIMER_1,            // TIMER
      .freq_hz = 50,                        // Hz
      .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&ledc_timer);

  const ledc_channel_config_t ledc_channel_pan = {
      .gpio_num = pwm_PAN, // GPIO
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_2,      // CHANNEL
      .intr_type = LEDC_INTR_DISABLE, // INTERUPT
      .timer_sel = LEDC_TIMER_1,      // TIMER
      .duty = 1229,                   // OVERFLOW VALUE
      .hpoint = 0};
  ledc_channel_config(&ledc_channel_pan);

  const ledc_channel_config_t ledc_channel_tilt = {
      .gpio_num = pwm_TILT, // GPIO
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_3,      // CHANNEL
      .intr_type = LEDC_INTR_DISABLE, // INTERUPT
      .timer_sel = LEDC_TIMER_1,      // TIMER
      .duty = 1229,                   // OVERFLOW VALUE
      .hpoint = 0};
  ledc_channel_config(&ledc_channel_tilt);
}

void setup()
{
  // setup wifi
  Serial.begin(115200);

  IPAddress ip(192, 168, 0, 245); // đây là IP tĩnh tự chọn trong UIT Public
  IPAddress gw(192, 168, 0, 1);
  IPAddress mask(255, 255, 255, 0);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.config(ip, gw, mask);

  if (WIFI_PASS != nullptr && WIFI_PASS[0] != '\0')
  {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  else
  {
    WiFi.begin(WIFI_SSID);
  }

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(300);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);

  // MOTOR
  setup_gpio(PWMA);
  setup_gpio(AIN2);
  setup_gpio(AIN1);
  setup_gpio(STBY);
  setup_gpio(BIN1);
  setup_gpio(BIN2);
  setup_gpio(PWMB);

  // SERVO
  setup_gpio(PAN);
  setup_gpio(TILT);

  // Kích hoạt driver, chỗ này phải cần cấp nguồn cho chân VCC (trải nghiệm)
  gpio_set_level(STBY, 1);

  // setup PWM_motor, dùng timer_0, channel_0/1
  setup_pwm_motor(PWMA, PWMB);

  // set tốc độ mặc định ban đầu là 0
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

  // setup PWM_servo, dùng timer_1, channel_2/3
  setup_pwm_servo(PAN, TILT);

  // set hướng xoay bánh cố định vì xe chỉ đi tiến nhanh chậm. có thể nâng cấp sau
  gpio_set_level(AIN1, 1);
  gpio_set_level(AIN2, 0);
  gpio_set_level(BIN1, 1);
  gpio_set_level(BIN2, 0);
}

void change_motor_speed()
{
  // scale lại để dùng tính duty, âm -> vật lệch trái, dương -> vật lệch phải
  float pan_loss_normalized = (pan_angle - 90.0) / 90.0;

  // công thức thay đổi speed theo loss
  // ví dụ: lệch -0.5 (trái), scale bánh trái chậm 50% và bánh phải nhanh lên 150%
  float speed_a = speed_limit * (1 - pan_loss_normalized);
  float speed_b = -speed_limit * (-1.0 - pan_loss_normalized);

  uint32_t duty_a = uint32_t(speed_a * MAX_DUTY_MOTOR);
  uint32_t duty_b = uint32_t(speed_b * MAX_DUTY_MOTOR);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_a);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_b);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void change_servor_angle()
{
  // servo MG996R góc 0-(0.5ms), 90-(1.5ms), 180-(2.5ms) -> duty = duty_per_ms * pulse_witdh
  float duty_per_ms = MAX_DUTY_SERVO / 20.0;
  float pan_pulse_witdh = (pan_angle / 90.0) + 0.5;
  float tilt_pulse_witdh = (tilt_angle / 90.0) + 0.5;

  uint32_t pan_duty_val = duty_per_ms * pan_pulse_witdh;
  uint32_t tilt_duty_val = duty_per_ms * tilt_pulse_witdh;

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, pan_duty_val);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, tilt_duty_val);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

void loop()
{

  if (udp.parsePacket() >= 4)
  {
    uint8_t buf[4];
    int n = udp.read(buf, 4);
    if (n == 4 && buf[0] == 0xAA && buf[3] == (uint8_t)(buf[0] ^ buf[1] ^ buf[2]))
    {
      pan_angle = buf[1];
      tilt_angle = buf[2];
      change_servor_angle();
      change_motor_speed();
    }
  }
}

// quy ước: A là bánh trái và ngược lại
// quy ước: Pan là trục ngang - Tilt là trục dọc
// quy ước: xe lúc này chỉ đi thẳng nhanh hoặc chậm, muốn chậm hơn có thể giới hạn scale lại (đã add giới hạn 50% speed, có thêm Kp để chỉnh)
// quy ước: motor A-B -> channel_0/1, servo pan-tilt -> channel_2/3