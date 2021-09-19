#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#include "pico_display.hpp"

#include "MLX90640_API.h"

#include <sstream>
#include <iomanip>

using namespace pimoroni;
#define CAM_ON

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

// I2C
constexpr uint I2CBaudRate = 1000 * 1000; // Will break ThermalCameraFrameDurationUs if not 1 MHz

// Thermal Camera
constexpr uint8_t ThermalCameraI2CAddress = 0x33;
constexpr uint8_t ThermalCameraWidth = 24;
constexpr uint8_t ThermalCameraHeight = 32;
constexpr uint8_t ThermalCameraFPS = 4;
constexpr int64_t ThermalCameraFrameDurationUs = I2CBaudRate / ThermalCameraFPS;

using ThermalCameraEEPROMDataType = uint16_t;
constexpr size_t ThermalCameraEEPROMDataSize = 832;

using ThermalCameraFrameDataType = uint16_t;
constexpr size_t ThermalCameraFrameDataSize = 834;

constexpr size_t FinalTemperatureDataSize = 768;

// --- CONSTANTS END

// Globals
uint16_t buffer[PicoDisplay::WIDTH * PicoDisplay::HEIGHT];
PicoDisplay display(buffer);

ThermalCameraEEPROMDataType cameraEEPROMData[ThermalCameraEEPROMDataSize];
ThermalCameraFrameDataType cameraFrameData[ThermalCameraFrameDataSize];
float finalTemperatureData[FinalTemperatureDataSize];

// TODO adjustable?
float emissivity = 1;

void CrashWithError(const char* error, int errorCode) {
    const Point errorMessageOrigin(0, 0);
    
    char buffer[512] = {};
    sprintf(buffer, "%s: %d", error, errorCode);
    
    const std::string errorString = buffer;
    
    while (true) {
        display.set_pen(255, 0, 0);
        display.clear();
        
        display.set_pen(0, 0, 255);
        display.text(errorString, errorMessageOrigin, 255, 2);
        
        printf(errorString.data());
        printf("\n");
        
        display.update();
        sleep_ms(500);
    }
}

int main() {
    bi_decl(bi_program_description("Pico Thermal Camera"));
    
    stdio_init_all();
    
    adc_init();
    adc_gpio_init(BatteryVoltagePin);
    adc_select_input(ADCInputID);
    
    gpio_init(USBConnectedPin);
    gpio_set_dir(USBConnectedPin, GPIO_IN);
    
    i2c_init(i2c_default, I2CBaudRate);
    
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    
    display.init();
    display.set_backlight(100);
    
    sleep_ms(2000);
    printf("Starting...\n");

#ifdef CAM_ON
    uint8_t fpsModeID = 0;
    switch(ThermalCameraFPS){
    case 1:
        fpsModeID = 0x01;
        break;
    case 2:
        fpsModeID = 0x02;
        break;
    case 4:
        fpsModeID = 0x03;
        break;
    case 8:
        fpsModeID = 0x04;
        break;
    case 16:
        fpsModeID = 0x05;
        break;
    case 32:
        fpsModeID = 0x06;
        break;
    case 64:
        fpsModeID = 0x07;
        break;
    default:
        CrashWithError("Unsupported FPS value", 0);
    }
    
    const int fpsModeResult = MLX90640_SetRefreshRate(ThermalCameraI2CAddress, fpsModeID);
    if (fpsModeResult != 0) {
        CrashWithError("Failed to set the camera refresh rate", fpsModeResult);
    }
    
    const int chessModeResult = MLX90640_SetChessMode(ThermalCameraI2CAddress);
    if (chessModeResult != 0) {
        CrashWithError("Failed to set the camera to chess mode", chessModeResult);
    }
    
    const int eeDumpResult = MLX90640_DumpEE(ThermalCameraI2CAddress, cameraEEPROMData);
    if (eeDumpResult != 0) {
        CrashWithError("Failed to dump camera eeprom data", eeDumpResult);
    }
    
    uint16_t hey = 64;
    printf("HEY: %d\n\n", hey);
    for (size_t b = 0; b < ThermalCameraFrameDataSize; ++b) {
        //printf("0x%04d ", cameraFrameData[b]);
        printf("%d", cameraFrameData[b]);
        
        if ((b + 1) % 8 == 0) {
            printf("\n");
        }
    }
    
    paramsMLX90640 mlx90640Params;
    const int paramExtractResult = MLX90640_ExtractParameters(cameraEEPROMData, &mlx90640Params);
    if (paramExtractResult != 0) {
        CrashWithError("Problems when parsing camera eeprom data", paramExtractResult);
    }
#endif // CAM_ON
    
    int64_t lastSleepDuration = 0;
    uint64_t iter = 0;
    while (true) {
        const absolute_time_t start = get_absolute_time();
        
        display.set_pen(120, 40, 60);
        display.clear();
        
        const uint16_t rawADC = adc_read();
        const float voltage = static_cast<float>(rawADC) * BatteryConversionFactor;
        
        display.set_pen(0, 255, 0);
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << voltage << "V; Sleep: " << lastSleepDuration << "us; Iter: " << iter;
        iter++;
        
        const std::string voltageStr = ss.str();
        display.text(voltageStr, BatteryTextOrigin, 255, 1);
        
        if (gpio_get(USBConnectedPin)) {
            display.text(usbConnected, USBTextOrigin, 255, 1);
        } else {
            display.text(usbDiconnected, USBTextOrigin, 255, 1);
        }
        
#ifdef CAM_ON
        const int frameDataFetchResult = MLX90640_GetFrameData(ThermalCameraI2CAddress, cameraFrameData);
        if (frameDataFetchResult < 0) {
            display.update();
            CrashWithError("Failed to get the frame data", frameDataFetchResult);
        }

        const float ta = MLX90640_GetTa(cameraFrameData, &mlx90640Params);
        MLX90640_CalculateTo(cameraFrameData, &mlx90640Params, emissivity, ta, finalTemperatureData);

        MLX90640_BadPixelsCorrection((&mlx90640Params)->brokenPixels, finalTemperatureData, 1, &mlx90640Params);
        MLX90640_BadPixelsCorrection((&mlx90640Params)->outlierPixels, finalTemperatureData, 1, &mlx90640Params);
#endif // CAM_ON

        display.update();
        
        const absolute_time_t end = get_absolute_time();
        const int64_t deltaUs = absolute_time_diff_us(start, end);
        lastSleepDuration = ThermalCameraFrameDurationUs - deltaUs;
        if (lastSleepDuration > 0) {
            sleep_us(lastSleepDuration);
        } else {
            lastSleepDuration = 0;
        }
    }
}
