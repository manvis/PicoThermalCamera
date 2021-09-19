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
#include <cmath>
#include <cfloat>

using namespace pimoroni;
#define CAM_ON

// --- CONSTANTS
// I2C
constexpr uint I2CBaudRate = 1000 * 1000; // Will break ThermalCameraFrameDurationUs if not 1 MHz

// Thermal Camera
constexpr uint8_t ThermalCameraI2CAddress = 0x33;
constexpr uint8_t ThermalCameraWidth = 24;
constexpr uint8_t ThermalCameraHeight = 32;
constexpr uint8_t ThermalCameraFPS = 4;
constexpr int64_t ThermalCameraFrameDurationUs = I2CBaudRate / ThermalCameraFPS;

constexpr size_t ThermalCameraEEPROMDataSize = 832;
constexpr size_t ThermalCameraFrameDataSize = 834;

constexpr int32_t TemperatureSensorWidth = 24;
constexpr int32_t TemperatureSensorHeight = 32;
constexpr int32_t FinalTemperatureDataSize = TemperatureSensorWidth * TemperatureSensorHeight;

constexpr int32_t NearestScaleMult = 4;
constexpr int32_t HeatmapTopOffsetPixels = 3;

constexpr int32_t TextXOffset = NearestScaleMult * TemperatureSensorWidth + 4;
constexpr int32_t TextYOffset = HeatmapTopOffsetPixels;

struct ShortColor3 {
    constexpr ShortColor3(int16_t r, int16_t g, int16_t b) : r(r), g(g), b(b) {}
    
    int16_t r;
    int16_t g;
    int16_t b;
};

constexpr std::array<ShortColor3, 7> HeatmapColors = {
    ShortColor3(  0,   0,   0),
    ShortColor3(  0,   0, 255),
    ShortColor3(  0, 255,   0),
    ShortColor3(255, 255,   0),
    ShortColor3(255,   0,   0),
    ShortColor3(255,   0, 255),
    ShortColor3(255, 255, 255),
};

// ADC/Battery
constexpr float BatteryConversionFactor = 3 * 3.3f / (1 << 12);
constexpr uint8_t BatteryVoltagePin = 29;
constexpr uint8_t ADCInputID = 3; // 0..3, which corresponds to pins 26..29
const Point BatteryTextOrigin(TextXOffset, TextYOffset);

// USB
constexpr uint8_t USBConnectedPin = 24;
constexpr int32_t TextLineHeight = 7;
const Point USBTextOrigin(TextXOffset, TextLineHeight + TextYOffset);
const std::string UsbConnectedTxt   = "USB Power";
const std::string UsbDiconnectedTxt = "Battery Power";

// UI
const std::string HoldX = "Hold X - Mark Min";
const std::string HoldY = "Hold Y - Mark Max";

// --- CONSTANTS END

// Globals
std::array<uint16_t, PicoDisplay::WIDTH * PicoDisplay::HEIGHT> displayBuffer;
PicoDisplay display(displayBuffer.data());

std::array<uint16_t, ThermalCameraEEPROMDataSize> cameraEEPROMData;
std::array<uint16_t, ThermalCameraFrameDataSize> cameraFrameData;
std::array<float, FinalTemperatureDataSize> finalTemperatureData;
std::array<Pen, FinalTemperatureDataSize> heatmapPixels;

// TODO adjustable?
float heatmapMin = 5;
float heatmapMax = 50;
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
    
    const int eeDumpResult = MLX90640_DumpEE(ThermalCameraI2CAddress, cameraEEPROMData.data());
    if (eeDumpResult != 0) {
        CrashWithError("Failed to dump camera eeprom data", eeDumpResult);
    }
    
    uint16_t hey = 64;
    printf("HEY: %d\n\n", hey);
    for (size_t b = 0; b < cameraFrameData.size(); ++b) {
        //printf("0x%04d ", cameraFrameData[b]);
        printf("%d", cameraFrameData[b]);
        
        if ((b + 1) % 8 == 0) {
            printf("\n");
        }
    }
    
    paramsMLX90640 mlx90640Params;
    const int paramExtractResult = MLX90640_ExtractParameters(cameraEEPROMData.data(), &mlx90640Params);
    if (paramExtractResult != 0) {
        CrashWithError("Problems when parsing camera eeprom data", paramExtractResult);
    }
#endif // CAM_ON

    const Pen textColor = display.create_pen(0, 255, 0);
    const Pen whitePen = display.create_pen(255, 255, 255);
    const Pen blackPen = display.create_pen(  0,   0,   0);
    
    int64_t lastSleepDuration = 0;
    uint64_t iter = 0;
    
    while (true) {
        const absolute_time_t start = get_absolute_time();
        
        display.set_pen(120, 40, 60);
        display.clear();
        
        const uint16_t rawADC = adc_read();
        const float voltage = static_cast<float>(rawADC) * BatteryConversionFactor;
        
        display.set_pen(textColor);
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << voltage << "V";
        iter++;
        
        const std::string voltageStr = ss.str();
        display.text(voltageStr, BatteryTextOrigin, 255, 1);
        
        if (gpio_get(USBConnectedPin)) {
            display.text(UsbConnectedTxt, USBTextOrigin, 255, 1);
        } else {
            display.text(UsbDiconnectedTxt, USBTextOrigin, 255, 1);
        }
        
#ifdef CAM_ON
        const int frameDataFetchResult = MLX90640_GetFrameData(ThermalCameraI2CAddress, cameraFrameData.data());
        if (frameDataFetchResult < 0) {
            display.update();
            CrashWithError("Failed to get the frame data", frameDataFetchResult);
        }

        const float ta = MLX90640_GetTa(cameraFrameData.data(), &mlx90640Params);
        MLX90640_CalculateTo(cameraFrameData.data(), &mlx90640Params, emissivity, ta, finalTemperatureData.data());

        MLX90640_BadPixelsCorrection((&mlx90640Params)->brokenPixels, finalTemperatureData.data(), 1, &mlx90640Params);
        MLX90640_BadPixelsCorrection((&mlx90640Params)->outlierPixels, finalTemperatureData.data(), 1, &mlx90640Params);
        
        const float heatmapRange = heatmapMax - heatmapMin;
        constexpr size_t lastColorID = HeatmapColors.size() - 1;
        
        float minTemp = FLT_MAX;
        Point minTempPixel;
        
        float maxTemp = -FLT_MIN;
        Point maxTempPixel;
        
        for (size_t x = 0; x < TemperatureSensorWidth; ++x) {
            for (size_t y = 0; y < TemperatureSensorHeight; ++y) {
                float value = finalTemperatureData[TemperatureSensorHeight * (TemperatureSensorWidth - 1 - x) + y];
                
                if (value < minTemp) {
                    minTemp = value;
                    minTempPixel = Point(x, y);
                }
                
                if (value > maxTemp) {
                    maxTemp = value;
                    maxTempPixel = Point(x, y);
                }
                
                value -= heatmapMin;
                value /= heatmapRange;
                
                float lerpDist = 0;
                
                size_t p0ID;
                size_t p1ID;
                if (value <= 0) {
                    p0ID = 0;
                    p1ID = 0;
                } else if (value >= 1) {
                    p0ID = lastColorID;
                    p1ID = lastColorID;
                } else {
                    value *= lastColorID;
                    p0ID = std::floor(value);
                    p1ID = p0ID + 1;
                    
                    lerpDist = value - static_cast<float>(p0ID);
                }
                
                const uint8_t r = ((HeatmapColors[p1ID].r - HeatmapColors[p0ID].r) * lerpDist) + HeatmapColors[p0ID].r;
                const uint8_t g = ((HeatmapColors[p1ID].g - HeatmapColors[p0ID].g) * lerpDist) + HeatmapColors[p0ID].g;
                const uint8_t b = ((HeatmapColors[p1ID].b - HeatmapColors[p0ID].b) * lerpDist) + HeatmapColors[p0ID].b;
                
                heatmapPixels[y * TemperatureSensorWidth + x] = display.create_pen(r, g, b);
            }
        }
        
        if (display.is_pressed(PicoDisplay::X)) {
            heatmapPixels[minTempPixel.y * TemperatureSensorWidth + minTempPixel.x] = blackPen;
        }

        if (display.is_pressed(PicoDisplay::Y)) {
            heatmapPixels[maxTempPixel.y * TemperatureSensorWidth + maxTempPixel.x] = whitePen;
        }

        
        for (int32_t x = 0; x < TemperatureSensorWidth; ++x) {
            for (int32_t y = 0; y < TemperatureSensorHeight; ++y) {
                const Pen p = heatmapPixels[y * TemperatureSensorWidth + x];
                
                for (int32_t r = 0; r < NearestScaleMult; ++r) {
                    display.set_pen(p);
                    display.pixel_span(Point(x * NearestScaleMult, (y * NearestScaleMult) + HeatmapTopOffsetPixels + r), NearestScaleMult);
                }
            }
        }
        
        display.set_pen(textColor);
        int32_t infoYOffset = USBTextOrigin.y + TextLineHeight * 2;
        
        char minValText[64];
        sprintf(minValText, "MIN: %.2fC", minTemp);
        display.text(minValText, Point(TextXOffset, infoYOffset), 255, 1);
        infoYOffset += TextLineHeight;
        
        char minValTextPixel[64];
        sprintf(minValTextPixel, "MIN PX: %d %d", minTempPixel.x, minTempPixel.y);
        display.text(minValTextPixel, Point(TextXOffset, infoYOffset), 255, 1);
        infoYOffset += TextLineHeight;
        display.text(HoldX, Point(TextXOffset, infoYOffset), 255, 1);
        infoYOffset += TextLineHeight * 2;
        
        char maxValText[64];
        sprintf(maxValText, "MAX: %.2fC", maxTemp);
        display.text(maxValText, Point(TextXOffset, infoYOffset), 255, 1);
        infoYOffset += TextLineHeight;
        
        char maxValTextPixel[64];
        sprintf(maxValTextPixel, "MAX PX: %d %d", maxTempPixel.x, maxTempPixel.y);
        display.text(maxValTextPixel, Point(TextXOffset, infoYOffset), 255, 1);
        infoYOffset += TextLineHeight;
        display.text(HoldY, Point(TextXOffset, infoYOffset), 255, 1);
        infoYOffset += TextLineHeight * 2;
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
