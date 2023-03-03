#include <Wire.h>        
#include "Arduino.h"       
#include "HT_SSD1306Wire.h"
#include <stdarg.h>

#include <SoftwareSerial.h>

#define MYPORT_RX 4
#define MYPORT_TX 3

/// Using software serial because problems with
// hw serial
SoftwareSerial s;// =  SoftwareSerial(rxPin, txPin);

SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

//19200
#define SERIAL_BAUDS_RATE 9600
#define REFRESH_INTERVAL 300
#define MAX_WAIT 3000
#define PWREN 2
#define BATPIN 37
#define SLEEP_TIMEOUT 120000
#define BUTTON_PIN 0
#define LED_PIN 35
#define RELATIVE_VALUE 0
#define VOLTAGE_DIVIDER  4.90 //((100.0 + 390.0)/100.0)

struct state_type {
  int distance_millimeters;
  bool laser_active;
  double voltage;
  bool switch_laser_on_off;
  long init_time;
  long last;
};

// Global var to hold the state
state_type state;


void message_s(int y, const char * format, ...) {
  char buff[1024];
  va_list va;
  va_start(va, format);
  vsprintf(buff, format, va);
  va_end(va);

  display.setFont(ArialMT_Plain_10);
  display.drawString(0, y, buff);
}

void message(int y, const char * msg) {
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, y, msg);
}

int read_measurement() {
  byte read_buffer[13];
  byte m[13];
  int count_wait_aa = 0;
  int num_bytes_read = 0;
  int aa = 0;

  // wait to align with message init
  do {
    if (s.available() > 0) {
      aa = s.read();
      num_bytes_read ++;
    } else {
      delay(1);
    }
    count_wait_aa++;
  } while (aa != 0xAA && count_wait_aa < MAX_WAIT);

  if (aa != 0xAA) {
    return -num_bytes_read;
  }

  int available = 0;
  while (available < 12) {
    available = s.available();
  }

  int total = 0;
  while (total < 12) {
    int r = s.readBytes(read_buffer, 12 - total);
    for (int i = total; i < total + r; ++i) {
      m[i] = read_buffer[i];
    }
    total += r;
  }

  int measurement = (m[5] << 24) +
                    (m[6] << 16) +
                    (m[7] << 8) +
                     m[8];

  return measurement;
}

void vext_on(int vext) {
  pinMode(vext,OUTPUT);
  digitalWrite(vext, LOW);
}

void vext_off(int vext) {
  pinMode(vext,OUTPUT);
  digitalWrite(vext, HIGH);

}

void init_screen() {
  vext_on(Vext);
  delay(10);
  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "Cortinasmalaga" );
  display.display();
  display.setFont(ArialMT_Plain_24);
}

void off_screen() {
  vext_off(Vext);
}

void init_serial() {
  s.begin(SERIAL_BAUDS_RATE, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
  while (!s) {
    delay(100);
    display.drawString(0, 24, "Wait for serial");
    display.display();
  }
}

void init_laser(int pwren_pin) {
  digitalWrite(pwren_pin, HIGH);
  delay(100);
  s.write(0x55);
  delay(200);
}

void show_measurement(int distance_millimeters) {
  char str[100];
  int m = distance_millimeters / 1000;
  int cm = (distance_millimeters  % 1000) / 10;
  int mm = distance_millimeters % 10;

  if (m > 0) {
    sprintf(str, "%d m %d cm", m, cm);
  } else {
    sprintf(str, "%3d   cm", cm);
  }
  message(2, str);
  sprintf(str, "%3d   mm", mm);
  message(30, str);
}

void start_continuous_slow_measurement() {
  byte continuous_slow_distance_measure[9] = {0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x05, 0x26};
  int n = s.write(continuous_slow_distance_measure, 9);
}

void start_continuous_measurement() {
  byte continuous_distance_measure[9] = {0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x25};
  int n = s.write(continuous_distance_measure, 9);
}

void stop_continous_measurement() {
  s.write(0x58);
}

void laser_off(int pin) {
  digitalWrite(pin, LOW);
}


// interrupt service routine
// REMEMBER not to add blocking stuff here
void laser_on_off_interrupt() {
    state.switch_laser_on_off = true;
}

void disable_leds() {
  digitalWrite(35, LOW);
}

void go_to_deep_sleep() {
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
  esp_deep_sleep_start();
}

void update_state() {
  if ((millis() - state.init_time) > SLEEP_TIMEOUT || state.switch_laser_on_off) {
    state.switch_laser_on_off = false;
    // wait before going to sleep to wait for the button to be stable
    delay(100);
    go_to_deep_sleep();
  }

  int last_measurement = -1;
  last_measurement = read_measurement();
  if (last_measurement > 0) {
    state.distance_millimeters = last_measurement;
  }

  // TODO this is not working
  state.voltage = (double)analogRead(1)*VOLTAGE_DIVIDER /1000.0;
}

void setup() {
  state = {0, true, 0.0, false, millis(), millis()};
  disable_leds();
  pinMode(PWREN, OUTPUT);

  init_screen();
  init_serial();
  init_laser(PWREN);
  start_continuous_measurement();

  adcAttachPin(1);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN),
                  laser_on_off_interrupt,
                  FALLING);

  analogReadResolution(12);
}

void loop() {
  update_state();

  if ( (millis() - state.last) > REFRESH_INTERVAL) {
    state.last = millis();
    display.clear();
    show_measurement(state.distance_millimeters - RELATIVE_VALUE);
    message_s(54, "                         %02d %.01fv",RELATIVE_VALUE, state.voltage);
    display.display();
  }
}