#include "SPS30.h"
#include "../interfaces/I2CSensor.h"
#include "../interfaces/endianness.h"
#include "furi_hal.h"
//#include <3rdparty/everest/include/everest/kremlin/c_endianness.h>

typedef union {
    uint16_t array16[2];
    uint8_t array8[4];
    float value;
} ByteToFl;

#ifndef UNITEMP_SPS30
bool unitemp_SPS30_alloc(Sensor* sensor, char* args);
bool unitemp_SPS30_init(Sensor* sensor);
bool unitemp_SPS30_deinit(Sensor* sensor);
UnitempStatus unitemp_SPS30_update(Sensor* sensor);
bool unitemp_SPS30_free(Sensor* sensor);
#endif


const SensorType SPS30 = {
    .typename = "SPS30",
    .interface = &I2C,
    .datatype = UT_DATA_TYPE_PM,
    .pollingInterval = 2000,
    .allocator = unitemp_SPS30_alloc,
    .mem_releaser = unitemp_SPS30_free,
    .initializer = unitemp_SPS30_init,
    .deinitializer = unitemp_SPS30_deinit,
    .updater = unitemp_SPS30_update};

#define SPS30_ID 0x69

// un-used commands
#define SPS_CMD_START_MEASUREMENT 0x0010
#define SPS_CMD_START_MEASUREMENT_ARG 0x0300
#define SPS_CMD_START_STOP_DELAY_USEC 20000
#define SPS_CMD_GET_SERIAL 0xd033
#define SPS_CMD_SLEEP 0x1001
#define SPS_CMD_READ_DEVICE_STATUS_REG 0xd206
#define SPS_CMD_START_MANUAL_FAN_CLEANING 0x5607
#define SPS_CMD_WAKE_UP 0x1103
#define SPS_CMD_DELAY_USEC 5000
#define SPS_CMD_DELAY_WRITE_FLASH_USEC 20000

// used commands
#define COMMAND_CONTINUOUS_MEASUREMENT 0x0010
#define COMMAND_GET_DATA_READY 0x0202
#define COMMAND_READ_MEASUREMENT 0x0300
#define COMMAND_SET_AUTOCLEAN_INTERVAL 0x8004
#define COMMAND_RESET 0xD304
#define COMMAND_STOP_MEASUREMENT 0x0104
#define COMMAND_READ_FW_VER 0xD100

static bool dataAvailable(Sensor* sensor) __attribute__((unused));
static bool readMeasurement(Sensor* sensor) __attribute__((unused));
static void reset(Sensor* sensor) __attribute__((unused));

static bool getFirmwareVersion(Sensor* sensor, uint16_t* val) __attribute__((unused));

static bool startMeasurement(Sensor* sensor) __attribute__((unused));
static bool stopMeasurement(Sensor* sensor) __attribute__((unused));


bool unitemp_SPS30_alloc(Sensor* sensor, char* args) {
    UNUSED(args);
    I2CSensor* i2c_sensor = (I2CSensor*)sensor->instance;

    i2c_sensor->minI2CAdr = SPS30_ID << 1;
    i2c_sensor->maxI2CAdr = SPS30_ID << 1;
    return true;
}

bool unitemp_SPS30_free(Sensor* sensor) {
    UNUSED(sensor);
    return true;
}

bool unitemp_SPS30_init(Sensor* sensor) {
    furi_hal_power_enable_otg();
    furi_delay_ms(100);

    if(startMeasurement(sensor) == true) {
    } else {
        return false;
    }

    return true;
}

bool unitemp_SPS30_deinit(Sensor* sensor) {
    bool retVal = stopMeasurement(sensor);
    furi_delay_ms(100);
    furi_hal_power_disable_otg();
    return retVal;
}

UnitempStatus unitemp_SPS30_update(Sensor* sensor) {
    readMeasurement(sensor);
    return UT_SENSORSTATUS_OK;
}

static uint8_t computeCRC8(uint8_t* message, uint8_t len) {
    uint8_t crc = 0xFF; // Init with 0xFF
    for (uint8_t x = 0; x < len; x++) {
        crc ^= message[x]; // XOR-in the next input byte
        for (uint8_t i = 0; i < 8; i++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }
    return crc; // No output reflection
}

// Sends a command along with arguments and CRC
static bool sendCommandWithCRC(Sensor* sensor, uint16_t command, uint16_t arguments) {
    static const uint8_t cmdSize = 5;

    uint8_t bytes[cmdSize];
    uint8_t *pointer = bytes;
    store16_be(pointer, command);
    pointer += 2;
    uint8_t *argPos = pointer;
    store16_be(pointer, arguments);
    pointer += 2;
    *pointer = computeCRC8(argPos, pointer - argPos);

    I2CSensor* i2c_sensor = (I2CSensor*)sensor->instance;
    return unitemp_i2c_writeArray(i2c_sensor, cmdSize, bytes);
}

// Sends just a command, no arguments, no CRC
static bool sendCommand(Sensor* sensor, uint16_t command) {
    static const uint8_t cmdSize = 2;

    uint8_t bytes[cmdSize];
    store16_be(bytes, command);

    I2CSensor* i2c_sensor = (I2CSensor*)sensor->instance;
    return unitemp_i2c_writeArray(i2c_sensor, cmdSize, bytes);
}

static uint16_t readRegister(Sensor* sensor, uint16_t registerAddress) {
    static const uint8_t regSize = 2;

    if(!sendCommand(sensor, registerAddress))
        return 0; // Sensor did not ACK

    furi_delay_ms(3);

    uint8_t bytes[regSize];
    I2CSensor* i2c_sensor = (I2CSensor*)sensor->instance;
    if(!unitemp_i2c_readArray(i2c_sensor, regSize, bytes))
        return 0;

    return load16_be(bytes);
}

static bool loadWord(uint8_t* buff, uint16_t* val) {
    uint16_t tmp = load16_be(buff);
    uint8_t expectedCRC = computeCRC8(buff, 2);
    if(buff[2] != expectedCRC)
        return false;
    *val = tmp;
    return true;
}

static bool getSettingValue(Sensor* sensor, uint16_t registerAddress, uint16_t* val) {
    static const uint8_t respSize = 3;

    if(!sendCommand(sensor, registerAddress))
        return false; // Sensor did not ACK

    furi_delay_ms(3);

    uint8_t bytes[respSize];
    I2CSensor* i2c_sensor = (I2CSensor*)sensor->instance;
    if(!unitemp_i2c_readArray(i2c_sensor, respSize, bytes))
        return false;

    return loadWord(bytes, val);
}

static bool loadFloat(uint8_t* buff, float* val) {
//    ByteToFl tmp;
    size_t cntr = 0;
    uint8_t floatBuff[4];
    for(size_t i = 0; i < 2; i++) {
        floatBuff[cntr++] = buff[0];
        floatBuff[cntr++] = buff[1];
        uint8_t expectedCRC = computeCRC8(buff, 2);
        if(buff[2] != expectedCRC) return false;
        buff += 3;
    }
    uint32_t tmpVal = load32_be(floatBuff);
    // Use memcpy to avoid breaking strict-aliasing rules
    memcpy(val, &tmpVal, sizeof(float));
    return true;
}

// Get 60 bytes from SPS30
// Updates global variables with floats
// Returns true if success
static bool readMeasurement(Sensor* sensor) {
    // Verify we have data from the sensor
    if(!dataAvailable(sensor)) {
        return false;
    }

    if(!sendCommand(sensor, COMMAND_READ_MEASUREMENT)) {
        FURI_LOG_E(APP_NAME, "Sensor did not ACK");
        return false; // Sensor did not ACK
    }

    furi_delay_ms(3);

    static const uint8_t respSize = 60;
    uint8_t buff[respSize];
    uint8_t* bytes = buff;
    I2CSensor* i2c_sensor = (I2CSensor*)sensor->instance;
    if(!unitemp_i2c_readArray(i2c_sensor, respSize, bytes)) {
        FURI_LOG_E(APP_NAME, "Error while read measures");
        return false;
    }

    bool error = false;
    #define LOAD_FLOAT_AND_UPDATE_SENSOR(bytes, sensorField, logMessage) \
        do { \
            float tempValue; \
            if(loadFloat(bytes, &tempValue)) { \
                sensor->sensorField = tempValue; \
            } else { \
                FURI_LOG_E(APP_NAME, logMessage); \
                error = true; \
            } \
            bytes += 6; \
        } while(0)

    // This order must be maintained as each macro internally increments the bytes counter
    LOAD_FLOAT_AND_UPDATE_SENSOR(bytes, mc_1p0, "Error while parsing MC 1.0");
    LOAD_FLOAT_AND_UPDATE_SENSOR(bytes, mc_2p5, "Error while parsing MC 2.5");
    LOAD_FLOAT_AND_UPDATE_SENSOR(bytes, mc_4p0, "Error while parsing MC 4.0");
    LOAD_FLOAT_AND_UPDATE_SENSOR(bytes, mc_10p0, "Error while parsing MC 10.0");
    LOAD_FLOAT_AND_UPDATE_SENSOR(bytes, nc_0p5, "Error while parsing NC 0.5");
    LOAD_FLOAT_AND_UPDATE_SENSOR(bytes, nc_1p0, "Error while parsing NC 1.0");
    LOAD_FLOAT_AND_UPDATE_SENSOR(bytes, nc_2p5, "Error while parsing NC 2.5");
    LOAD_FLOAT_AND_UPDATE_SENSOR(bytes, nc_4p0, "Error while parsing NC 4.0");
    LOAD_FLOAT_AND_UPDATE_SENSOR(bytes, nc_10p0, "Error while parsing NC 10.0");
    LOAD_FLOAT_AND_UPDATE_SENSOR(bytes, typical_particle_size, "Error while parsing typical_particle_size");

    return !error;
}

static void reset(Sensor* sensor) {
    sendCommand(sensor, COMMAND_RESET);
}

static bool getFirmwareVersion(Sensor* sensor, uint16_t* val) {
    return getSettingValue(sensor, COMMAND_READ_FW_VER, val);
}

// Stop continuous measurement
static bool stopMeasurement(Sensor* sensor) {
    return sendCommand(sensor, COMMAND_STOP_MEASUREMENT);
}

// Sets interval between measurements
// 2 seconds to 1800 seconds (30 minutes)
static bool startMeasurement(Sensor* sensor) {
    uint16_t test = 0;
    if (getFirmwareVersion(sensor, &test)) {
        sensor->mc_2p5 = 15;
    } else {
        sensor->mc_2p5 = 16;
    }
    if(!sendCommandWithCRC(sensor, COMMAND_CONTINUOUS_MEASUREMENT, 0x0300))
        return false;
    // TODO: check if this worked?
    return true;
}

// Returns true when data is available
static bool dataAvailable(Sensor* sensor) {
    return 1 == readRegister(sensor, COMMAND_GET_DATA_READY);
}