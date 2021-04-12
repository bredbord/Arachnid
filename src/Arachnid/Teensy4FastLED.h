// Controller Header File Based upon the PJRC Forum Post by user "spolsky"
// https://forum.pjrc.com/threads/63231-FastLED-with-Teensy-4-1-fast-parallel-DMA-output-on-any-pin
// Accessed on 3/22/2021
// Larson Rivera

#ifndef TEENSY_FASTLED_BRIDGE
#define TEENSY_FASTLED_BRIDGE

#include <Arduino.h>
#include <OctoWS2811.h>
#include <FastLED.h>

template <EOrder COLOR_ORDER, unit8_t CHIP_TYPE>
class Teensy4FastLED : public CPixelLEDController<COLOR_ORDER, 8, 0xFF> {
  private:
    OctoWS2811 *OctoWSptr;

  public:
    Teensy4FastLED(OctoWS2811 *externalOctoWSptr) { this->OctoWSptr = externalOctoWSptr; }

    virtual void init() {};

    virtual void showPixels(PixelController<COLOR_ORDER, 8, 0xFF> &pixels) {
      uint32_t i = 0;

      while (pixels.has(1)) {
        uint8_t r = pixels.loadAndScale0();
        uint8_t g = pixels.loadAndScale1();
        uint8_t b = pixels.loadAndScale2();
        
        this->OctoWSptr->setPixel(i++, r, g, b);
        pixels.stepDithering();
        pixels.advanceData();
      }

      this->OctoWSptr->show();
    }
};

#endif
