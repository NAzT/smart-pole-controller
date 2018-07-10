#define CMMC_USE_ALIAS

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <CMMC_SimplePair.h>
#include <CMMC_Utils.h>
#include <CMMC_ESPNow.h>
#include <CMMC_LED.h>
#include <CMMC_BootMode.h>
#include "data_type.h"

u8 isCrashed = 0; 
#include <SoftwareSerial.h>
#define rxPin 14
#define txPin 12

bool shouldSendPacketOverESPNowBack = false;

// SoftwareSerial swSerial(rxPin, txPin, false, 128);
SoftwareSerial swSerial(rxPin, txPin);

#define LED_PIN                 2
#define BUTTON_PIN              0
#define PROD_MODE_PIN         13

int mode;
bool isIgnoreNewMessage = 0;

CMMC_SimplePair instance;
CMMC_ESPNow espNow;
CMMC_Utils utils;
CMMC_LED led(LED_PIN, LOW);

uint8_t targetMacAddr[6];

void evt_callback(u8 status, u8* sa, const u8* data) {
  if (status == 0) {
    Serial.printf("[CSP_EVENT_SUCCESS] STATUS: %d\r\n", status);
    Serial.printf("WITH KEY: ");
    utils.dump(data, 16);
    Serial.printf("WITH MAC: ");
    utils.dump(sa, 6);
    ESP.reset();
  }
  else {
    Serial.printf("[CSP_EVENT_ERROR] %d: %s\r\n", status, (const char*)data);
  }
}
void setup_hardware() {
  Serial.begin(57600);
  swSerial.begin(9600);

  Serial.println();
  led.init();
}

void start_config_mode() {
  uint8_t* controller_addr = utils.getESPNowControllerMacAddress();
  utils.printMacAddress(controller_addr);
  instance.begin(MASTER_MODE, evt_callback);
  instance.debug([](const char* s) {
    Serial.println(s);
  });
  instance.set_message(controller_addr, 6);
  instance.start();
}


int counter = 0;

#include <CMMC_RX_Parser.h>
CMMC_RX_Parser parser(&swSerial);

uint32_t time;


void setup()
{
  setup_hardware();
  Serial.println("Controller Mode");
  pinMode(PROD_MODE_PIN, INPUT_PULLUP); 
  uint32_t wait_config = 1000;
  if (digitalRead(PROD_MODE_PIN) == LOW) {
    wait_config = 0; 
  } 
  Serial.printf("wait_config = %d \r\n", wait_config); 
  CMMC_BootMode bootMode(&mode, BUTTON_PIN);
  bootMode.init();
  bootMode.check([](int mode) {
    Serial.printf("done.... mode = %d \r\n", mode);
    if (mode == BootMode::MODE_CONFIG) {
      start_config_mode();
    }
    else if (mode == BootMode::MODE_RUN) {
      Serial.print("Initializing... Controller..");
      espNow.init(NOW_MODE_CONTROLLER);
      espNow.on_message_recv([](uint8_t *macaddr, uint8_t *data, uint8_t len) {
        Serial.print("FROM: ");
        CMMC::dump(macaddr, 6); 
        memcpy(targetMacAddr, macaddr, 6);
        if (data[0] && !isIgnoreNewMessage) {
          isCrashed = data[0];
          led.low();
        }
        else {
          led.high(); 
        }
        shouldSendPacketOverESPNowBack = true; 
      });

      espNow.on_message_sent([](uint8_t *macaddr,  uint8_t status) {
        shouldSendPacketOverESPNowBack = false;
      });
    }
    else {
      // unhandled
    }
  }, wait_config);
}

#include <CMMC_TimeOut.h>
CMMC_TimeOut ct;
void loop()
{
  while (mode == BootMode::MODE_CONFIG) {
    ct.timeout_ms(60000);
    while (1 && !ct.is_timeout()) {
      delay(500);
      led.toggle();
    }
    Serial.println("Simple Pair Wait timeout.");
    ESP.reset();
  }

  parser.process();
  delay(1);
  if (isCrashed) {
    led.low();
  }
  else {
    led.high(); 
  }

  while (shouldSendPacketOverESPNowBack) {
    Serial.printf("SENT BYTE = %u \r\n", isCrashed);
    espNow.send(targetMacAddr, &isCrashed, 1);
    delay(1);
  }

  ct.timeout_ms(5000);
  while (digitalRead(13) == LOW) {
    if (ct.is_timeout()) {
      ESP.reset();
    }
    isCrashed = 0;
    led.high();
    uint32_t nextIgnoreMessage = millis() + 30*1000L;
    isIgnoreNewMessage = 1; 
    while(millis() < nextIgnoreMessage) {
      yield();
      while (shouldSendPacketOverESPNowBack) {
        Serial.printf("SENT BYTE = %u \r\n", isCrashed);
        espNow.send(targetMacAddr, &isCrashed, 1);
        delay(1);
      }
    }
    isIgnoreNewMessage = 0; 
  }
}
