#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/binary_info.h"

#include "pico_display.hpp"

#include <sstream>
#include <iomanip>

using namespace pimoroni;

// --- CONSTANTS
// ADC/Battery
constexpr float BatteryConversionFactor = 3 * 3.3f / (1 << 12);
constexpr uint8_t BatteryVoltagePin = 29;
constexpr uint8_t ADCInputID = 3; // 0..3, which corresponds to pins 26..29
const Point BatteryTextOrigin(0, 0);

// USB
constexpr uint8_t USBConnectedPin = 24;
const Point USBTextOrigin(0, 7);
const std::string usbConnected   = "USB Connected";
const std::string usbDiconnected = "USB Disconnected";
// --- CONSTANTS END

// Globals
uint16_t buffer[PicoDisplay::WIDTH * PicoDisplay::HEIGHT];
PicoDisplay display(buffer);

int main() {
    bi_decl(bi_program_description("Pico Thermal Camera"));
    
    stdio_init_all();
    
    adc_init();
    adc_gpio_init(BatteryVoltagePin);
    adc_select_input(ADCInputID);
    
    gpio_init(USBConnectedPin);
    gpio_set_dir(USBConnectedPin, GPIO_IN);
    
    display.init();
    display.set_backlight(100);
    
    while (true) {
        display.set_pen(120, 40, 60);
        display.clear();
        
        const uint16_t rawADC = adc_read();
        const float voltage = static_cast<float>(rawADC) * BatteryConversionFactor;
        
        display.set_pen(0, 255, 0);
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << voltage << "V";
        
        const std::string voltageStr = ss.str();
        display.text(voltageStr, BatteryTextOrigin, 255, 1);
        
        if (gpio_get(USBConnectedPin)) {
            display.text(usbConnected, USBTextOrigin, 255, 1);
        } else {
            display.text(usbDiconnected, USBTextOrigin, 255, 1);
        }
    
        display.update();
        sleep_ms(500);
    }
}
