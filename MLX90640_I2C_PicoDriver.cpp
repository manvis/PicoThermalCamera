/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "MLX90640_I2C_Driver.h"
#include "hardware/i2c.h"

#include <stdio.h>

void MLX90640_I2CInit()
{   
    //
}

int MLX90640_I2CGeneralReset(void)
{    
    return -1;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    constexpr size_t tempBufferSize = 1668;
    
    const int readSizeBytes = nMemAddressRead * 2;
    if (readSizeBytes > tempBufferSize) {
        printf("Requested read too big: %zu\n", tempBufferSize);
        return -3;
    }
    
    const uint8_t cmd[2] = {(uint8_t)(startAddress >> 8), (uint8_t)(startAddress & 0xFF)};
    
    uint8_t buf[tempBufferSize] = {0};
    uint16_t *p = data;
    
    const int writtenByteCount = i2c_write_blocking(i2c_default, slaveAddr, cmd, 2, true);
    if (writtenByteCount != 2) {
        printf("Incorrect written byte count: %d\n", writtenByteCount);
        return -2;
    }
    
    const int readByteCount = i2c_read_blocking(i2c_default, slaveAddr, buf, readSizeBytes, false);
    if (readByteCount != readSizeBytes) {
        printf("Incorrect read byte count: %d\n", readByteCount);
        return -1;
    }
    
    for (int count = 0; count < nMemAddressRead; ++count){
	    int i = count << 1;
    	*p++ = (((uint16_t)buf[i]) * 256) | ((uint16_t)buf[i+1]);
    }
    
    return 0;   
} 

void MLX90640_I2CFreqSet(int)
{
    //
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    const uint8_t cmd[4] = {(uint8_t)(writeAddress >> 8), (uint8_t)(writeAddress & 0x00FF), (uint8_t)(data >> 8), (uint8_t)(data & 0x00FF)};
    
    const int writtenByteCount = i2c_write_blocking(i2c_default, slaveAddr, cmd, 4, false);
    if (writtenByteCount != 4) {
        return -1;
    }
    
    uint16_t dataCheck;
    MLX90640_I2CRead(slaveAddr, writeAddress, 1, &dataCheck);
    if (dataCheck != data)
    {
        return -2;
    }
    
    return 0;
}

