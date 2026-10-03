#pragma once

#include <MeshCore.h>
#include <Arduino.h>
#include <helpers/NRF52Board.h>

#ifndef USER_BTN_PRESSED
#define USER_BTN_PRESSED LOW
#endif

#ifdef XIAO_NRF52

class XiaoNrf52Board : public NRF52BoardDCDC {
protected:
#if NRF52_POWER_MANAGEMENT
  void initiateShutdown(uint8_t reason) override;
#endif

public:
  XiaoNrf52Board() : NRF52Board("XIAO_NRF52_OTA") {}
  void begin();

#if defined(P_LORA_TX_LED)
  void onBeforeTransmit() override {
    digitalWrite(P_LORA_TX_LED, LOW);   // turn TX LED on
  }
  void onAfterTransmit() override {
    digitalWrite(P_LORA_TX_LED, HIGH);   // turn TX LED off
  }
#endif

  uint16_t getBattMilliVolts() override;

#ifdef PIN_BOARD_LIGHT
  // zahrada: vystup na MOSFET LR7843. Bit 0 = svetlo (1 = sviti).
  // Oficialni CLI 'io' (io / io s 1 / io r 1) i prikazy LON/LOFF jdou pres tyto dve funkce.
  uint32_t light_state = 0;
  void setGpio(uint32_t values) override {
    light_state = values & 1;
    digitalWrite(PIN_BOARD_LIGHT, light_state ? HIGH : LOW);
#ifdef OUTPUT_H0H1
    pinMode(PIN_BOARD_LIGHT, OUTPUT_H0H1);   // silnejsi budic vystupu (vstup modulu ma optoclen)
#else
    pinMode(PIN_BOARD_LIGHT, OUTPUT);
#endif
  }
  uint32_t getGpio() override { return light_state; }
#endif

  const char* getManufacturerName() const override {
    return "Seeed Xiao-nrf52";
  }

  void powerOff() override {
#ifdef PIN_BOARD_LIGHT
    setGpio(0);   // zahrada: pred vypnutim zhasnout (vystup by jinak zustal sepnuty i ve vypnutem stavu)
#endif
    // set led on and wait for button release before poweroff
    digitalWrite(PIN_LED, LOW);
#ifdef PIN_USER_BTN
    while(digitalRead(PIN_USER_BTN) == USER_BTN_PRESSED);
#endif
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(PIN_LED, HIGH);

#ifdef PIN_USER_BTN
    // configure button press to wake up when in powered off state
    nrf_gpio_cfg_sense_input(digitalPinToInterrupt(g_ADigitalPinMap[PIN_USER_BTN]), NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
#endif

    NRF52Board::powerOff();
  }
};

#endif