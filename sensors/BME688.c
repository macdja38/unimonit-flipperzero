/*
    Unitemp - Universal temperature reader
    Copyright (C) 2022-2023  Victor Nikitchuk (https://github.com/quen0n)
    Contributed by g0gg0 (https://github.com/g3gg0)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include "BME688.h"
#include "../views/UnitempViews.h"
#include <gui/modules/variable_item_list.h>
#include "unimonit_icons.h"

const SensorType BME688 = {
    .typename = "BME688",
    .interface = &I2C,
    .datatype = UT_TEMPERATURE | UT_HUMIDITY | UT_PRESSURE | UT_VOC,
    .pollingInterval = 500,
    .allocator = unitemp_BME688_alloc,
    .mem_releaser = unitemp_BME688_free,
    .initializer = unitemp_BME688_init,
    .deinitializer = unitemp_BME688_deinit,
    .updater = unitemp_BME688_update,
    .displayActions = unitemp_BME688_display_actions,
};

//Интервал обновления калибровочных значений
#define BOSCH_CAL_UPDATE_INTERVAL 60000

#define BME688_ID 0x61

#define BME688_I2C_ADDR_MIN (0x76 << 1)
#define BME688_I2C_ADDR_MAX (0x77 << 1)

#define BME688_REG_STATUS 0x1D
#define BME688_REG_CTRL_MEAS 0x74
#define BME688_REG_CONFIG 0x75
#define BME688_REG_CTRL_HUM 0x72
//Преддескретизация температуры
#define BME688_TEMP_OVERSAMPLING_SKIP 0b00000000
#define BME688_TEMP_OVERSAMPLING_1 0b00100000
#define BME688_TEMP_OVERSAMPLING_2 0b01000000
#define BME688_TEMP_OVERSAMPLING_4 0b01100000
#define BME688_TEMP_OVERSAMPLING_8 0b10000000
#define BME688_TEMP_OVERSAMPLING_16 0b10100000
//Преддескретизация давления
#define BME688_PRESS_OVERSAMPLING_SKIP 0b00000000
#define BME688_PRESS_OVERSAMPLING_1 0b00000100
#define BME688_PRESS_OVERSAMPLING_2 0b00001000
#define BME688_PRESS_OVERSAMPLING_4 0b00001100
#define BME688_PRESS_OVERSAMPLING_8 0b00010000
#define BME688_PRESS_OVERSAMPLING_16 0b00010100
//Преддескретизация влажности
#define BME688_HUM_OVERSAMPLING_SKIP 0b00000000
#define BME688_HUM_OVERSAMPLING_1 0b00000001
#define BME688_HUM_OVERSAMPLING_2 0b00000010
#define BME688_HUM_OVERSAMPLING_4 0b00000011
#define BME688_HUM_OVERSAMPLING_8 0b00000100
#define BME688_HUM_OVERSAMPLING_16 0b00000101
//Режимы работы датчика
#define BME688_MODE_SLEEP 0b00000000 //Наелся и спит
#define BME688_MODE_FORCED 0b00000001 //Обновляет значения 1 раз, после чего уходит в сон
//Коэффициент фильтрации значений
#define BME688_FILTER_COEFF_1 0b00000000
#define BME688_FILTER_COEFF_2 0b00000100
#define BME688_FILTER_COEFF_4 0b00001000
#define BME688_FILTER_COEFF_8 0b00001100
#define BME688_FILTER_COEFF_16 0b00010000
//Разрешить работу по SPI
#define BME688_SPI_3W_ENABLE 0b00000001
#define BME688_SPI_3W_DISABLE 0b00000000

/* https://github.com/boschsensortec/BME688_driver/blob/master/bme688.c or
   https://github.com/boschsensortec/BME68x-Sensor-API */
static float BME688_compensate_temperature(I2CSensor* i2c_sensor, int32_t temp_adc) {
    BME688_instance* bme688_instance = (BME688_instance*)i2c_sensor->sensorInstance;
    float var1 = 0;
    float var2 = 0;
    float calc_temp = 0;

    /* calculate var1 data */
    var1 =
        ((((float)temp_adc / 16384.0f) - ((float)bme688_instance->temp_cal.dig_T1 / 1024.0f)) *
         ((float)bme688_instance->temp_cal.dig_T2));

    /* calculate var2 data */
    var2 =
        (((((float)temp_adc / 131072.0f) - ((float)bme688_instance->temp_cal.dig_T1 / 8192.0f)) *
          (((float)temp_adc / 131072.0f) - ((float)bme688_instance->temp_cal.dig_T1 / 8192.0f))) *
         ((float)bme688_instance->temp_cal.dig_T3 * 16.0f));

    /* t_fine value*/
    bme688_instance->t_fine = (var1 + var2);

    /* compensated temperature data*/
    calc_temp = ((bme688_instance->t_fine) / 5120.0f);

    return calc_temp;
}

static float BME688_compensate_pressure(I2CSensor* i2c_sensor, int32_t pres_adc) {
    BME688_instance* bme688_instance = (BME688_instance*)i2c_sensor->sensorInstance;

    float var1;
    float var2;
    float var3;
    float calc_pres;

    var1 = (((float)bme688_instance->t_fine / 2.0f) - 64000.0f);
    var2 = var1 * var1 * (((float)bme688_instance->press_cal.dig_P6) / (131072.0f));
    var2 = var2 + (var1 * ((float)bme688_instance->press_cal.dig_P5) * 2.0f);
    var2 = (var2 / 4.0f) + (((float)bme688_instance->press_cal.dig_P4) * 65536.0f);
    var1 =
        (((((float)bme688_instance->press_cal.dig_P3 * var1 * var1) / 16384.0f) +
          ((float)bme688_instance->press_cal.dig_P2 * var1)) /
         524288.0f);
    var1 = ((1.0f + (var1 / 32768.0f)) * ((float)bme688_instance->press_cal.dig_P1));
    calc_pres = (1048576.0f - ((float)pres_adc));

    /* Avoid exception caused by division by zero */
    if((int)var1 != 0) {
        calc_pres = (((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1);
        var1 =
            (((float)bme688_instance->press_cal.dig_P9) * calc_pres * calc_pres) / 2147483648.0f;
        var2 = calc_pres * (((float)bme688_instance->press_cal.dig_P8) / 32768.0f);
        var3 =
            ((calc_pres / 256.0f) * (calc_pres / 256.0f) * (calc_pres / 256.0f) *
             (bme688_instance->press_cal.dig_P10 / 131072.0f));
        calc_pres =
            (calc_pres +
             (var1 + var2 + var3 + ((float)bme688_instance->press_cal.dig_P7 * 128.0f)) / 16.0f);
    } else {
        calc_pres = 0;
    }

    return calc_pres;
}

static float BME688_compensate_humidity(I2CSensor* i2c_sensor, int32_t hum_adc) {
    BME688_instance* bme688_instance = (BME688_instance*)i2c_sensor->sensorInstance;
    float calc_hum;
    float var1;
    float var2;
    float var3;
    float var4;
    float temp_comp;

    /* compensated temperature data*/
    temp_comp = ((bme688_instance->t_fine) / 5120.0f);
    var1 =
        (float)((float)hum_adc) - (((float)bme688_instance->hum_cal.dig_H1 * 16.0f) +
                                   (((float)bme688_instance->hum_cal.dig_H3 / 2.0f) * temp_comp));
    var2 = var1 *
           ((float)(((float)bme688_instance->hum_cal.dig_H2 / 262144.0f) *
                    (1.0f + (((float)bme688_instance->hum_cal.dig_H4 / 16384.0f) * temp_comp) +
                     (((float)bme688_instance->hum_cal.dig_H5 / 1048576.0f) * temp_comp * temp_comp))));
    var3 = (float)bme688_instance->hum_cal.dig_H6 / 16384.0f;
    var4 = (float)bme688_instance->hum_cal.dig_H7 / 2097152.0f;
    calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
    if(calc_hum > 100.0f) {
        calc_hum = 100.0f;
    } else if(calc_hum < 0.0f) {
        calc_hum = 0.0f;
    }

    return calc_hum;
}

/* https://github.com/boschsensortec/BME688_driver/blob/master/bme688_defs.h */
#define BME688_COEFF_SIZE UINT8_C(41)
#define BME688_COEFF_ADDR1_LEN UINT8_C(25)
#define BME688_COEFF_ADDR2_LEN UINT8_C(16)
#define BME688_COEFF_ADDR1 UINT8_C(0x89)
#define BME688_COEFF_ADDR2 UINT8_C(0xe1)
#define BME688_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)
#define BME688_T2_LSB_REG (1)
#define BME688_T2_MSB_REG (2)
#define BME688_T3_REG (3)
#define BME688_P1_LSB_REG (5)
#define BME688_P1_MSB_REG (6)
#define BME688_P2_LSB_REG (7)
#define BME688_P2_MSB_REG (8)
#define BME688_P3_REG (9)
#define BME688_P4_LSB_REG (11)
#define BME688_P4_MSB_REG (12)
#define BME688_P5_LSB_REG (13)
#define BME688_P5_MSB_REG (14)
#define BME688_P7_REG (15)
#define BME688_P6_REG (16)
#define BME688_P8_LSB_REG (19)
#define BME688_P8_MSB_REG (20)
#define BME688_P9_LSB_REG (21)
#define BME688_P9_MSB_REG (22)
#define BME688_P10_REG (23)
#define BME688_H2_MSB_REG (25)
#define BME688_H2_LSB_REG (26)
#define BME688_H1_LSB_REG (26)
#define BME688_H1_MSB_REG (27)
#define BME688_H3_REG (28)
#define BME688_H4_REG (29)
#define BME688_H5_REG (30)
#define BME688_H6_REG (31)
#define BME688_H7_REG (32)
#define BME688_T1_LSB_REG (33)
#define BME688_T1_MSB_REG (34)
#define BME688_GH2_LSB_REG (35)
#define BME688_GH2_MSB_REG (36)
#define BME688_GH1_REG (37)
#define BME688_GH3_REG (38)
#define BME688_HUM_REG_SHIFT_VAL UINT8_C(4)
#define BME688_BIT_H1_DATA_MSK UINT8_C(0x0F)

static bool BME688_readCalValues(I2CSensor* i2c_sensor) {
    BME688_instance* bme688_instance = (BME688_instance*)i2c_sensor->sensorInstance;
    uint8_t coeff_array[BME688_COEFF_SIZE] = {0};

    if(!unitemp_i2c_readRegArray(
           i2c_sensor, BME688_COEFF_ADDR1, BME688_COEFF_ADDR1_LEN, &coeff_array[0]))
        return false;
    if(!unitemp_i2c_readRegArray(
           i2c_sensor,
           BME688_COEFF_ADDR2,
           BME688_COEFF_ADDR2_LEN,
           &coeff_array[BME688_COEFF_ADDR1_LEN]))
        return false;

    /* Temperature related coefficients */
    bme688_instance->temp_cal.dig_T1 = (uint16_t)(BME688_CONCAT_BYTES(
        coeff_array[BME688_T1_MSB_REG], coeff_array[BME688_T1_LSB_REG]));
    bme688_instance->temp_cal.dig_T2 = (int16_t)(BME688_CONCAT_BYTES(
        coeff_array[BME688_T2_MSB_REG], coeff_array[BME688_T2_LSB_REG]));
    bme688_instance->temp_cal.dig_T3 = (int8_t)(coeff_array[BME688_T3_REG]);

    /* Pressure related coefficients */
    bme688_instance->press_cal.dig_P1 = (uint16_t)(BME688_CONCAT_BYTES(
        coeff_array[BME688_P1_MSB_REG], coeff_array[BME688_P1_LSB_REG]));
    bme688_instance->press_cal.dig_P2 = (int16_t)(BME688_CONCAT_BYTES(
        coeff_array[BME688_P2_MSB_REG], coeff_array[BME688_P2_LSB_REG]));
    bme688_instance->press_cal.dig_P3 = (int8_t)coeff_array[BME688_P3_REG];
    bme688_instance->press_cal.dig_P4 = (int16_t)(BME688_CONCAT_BYTES(
        coeff_array[BME688_P4_MSB_REG], coeff_array[BME688_P4_LSB_REG]));
    bme688_instance->press_cal.dig_P5 = (int16_t)(BME688_CONCAT_BYTES(
        coeff_array[BME688_P5_MSB_REG], coeff_array[BME688_P5_LSB_REG]));
    bme688_instance->press_cal.dig_P6 = (int8_t)(coeff_array[BME688_P6_REG]);
    bme688_instance->press_cal.dig_P7 = (int8_t)(coeff_array[BME688_P7_REG]);
    bme688_instance->press_cal.dig_P8 = (int16_t)(BME688_CONCAT_BYTES(
        coeff_array[BME688_P8_MSB_REG], coeff_array[BME688_P8_LSB_REG]));
    bme688_instance->press_cal.dig_P9 = (int16_t)(BME688_CONCAT_BYTES(
        coeff_array[BME688_P9_MSB_REG], coeff_array[BME688_P9_LSB_REG]));
    bme688_instance->press_cal.dig_P10 = (uint8_t)(coeff_array[BME688_P10_REG]);

    /* Humidity related coefficients */
    bme688_instance->hum_cal.dig_H1 =
        (uint16_t)(((uint16_t)coeff_array[BME688_H1_MSB_REG] << BME688_HUM_REG_SHIFT_VAL) | (coeff_array[BME688_H1_LSB_REG] & BME688_BIT_H1_DATA_MSK));
    bme688_instance->hum_cal.dig_H2 =
        (uint16_t)(((uint16_t)coeff_array[BME688_H2_MSB_REG] << BME688_HUM_REG_SHIFT_VAL) | ((coeff_array[BME688_H2_LSB_REG]) >> BME688_HUM_REG_SHIFT_VAL));
    bme688_instance->hum_cal.dig_H3 = (int8_t)coeff_array[BME688_H3_REG];
    bme688_instance->hum_cal.dig_H4 = (int8_t)coeff_array[BME688_H4_REG];
    bme688_instance->hum_cal.dig_H5 = (int8_t)coeff_array[BME688_H5_REG];
    bme688_instance->hum_cal.dig_H6 = (uint8_t)coeff_array[BME688_H6_REG];
    bme688_instance->hum_cal.dig_H7 = (int8_t)coeff_array[BME688_H7_REG];

    /* Gas heater related coefficients */
    bme688_instance->gas_cal.dig_GH1 = (int8_t)coeff_array[BME688_GH1_REG];
    bme688_instance->gas_cal.dig_GH2 = (int16_t)(BME688_CONCAT_BYTES(
        coeff_array[BME688_GH2_MSB_REG], coeff_array[BME688_GH2_LSB_REG]));
    bme688_instance->gas_cal.dig_GH3 = (int8_t)coeff_array[BME688_GH3_REG];

#ifdef UNITEMP_DEBUG
    FURI_LOG_D(
        APP_NAME,
        "Sensor BME688 T1-T3: %d, %d, %d",
        bme688_instance->temp_cal.dig_T1,
        bme688_instance->temp_cal.dig_T2,
        bme688_instance->temp_cal.dig_T3);

    FURI_LOG_D(
        APP_NAME,
        "Sensor BME688: P1-P10: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",
        bme688_instance->press_cal.dig_P1,
        bme688_instance->press_cal.dig_P2,
        bme688_instance->press_cal.dig_P3,
        bme688_instance->press_cal.dig_P4,
        bme688_instance->press_cal.dig_P5,
        bme688_instance->press_cal.dig_P6,
        bme688_instance->press_cal.dig_P7,
        bme688_instance->press_cal.dig_P8,
        bme688_instance->press_cal.dig_P9,
        bme688_instance->press_cal.dig_P10);

    FURI_LOG_D(
        APP_NAME,
        "Sensor BME688: H1-H7: %d, %d, %d, %d, %d, %d, %d",
        bme688_instance->hum_cal.dig_H1,
        bme688_instance->hum_cal.dig_H2,
        bme688_instance->hum_cal.dig_H3,
        bme688_instance->hum_cal.dig_H4,
        bme688_instance->hum_cal.dig_H5,
        bme688_instance->hum_cal.dig_H6,
        bme688_instance->hum_cal.dig_H7);

    FURI_LOG_D(
        APP_NAME,
        "Sensor BME688 GH1-GH3: %d, %d, %d",
        bme688_instance->gas_cal.dig_GH1,
        bme688_instance->gas_cal.dig_GH2,
        bme688_instance->gas_cal.dig_GH3);

#endif

    bme688_instance->last_cal_update_time = furi_get_tick();
    return true;
}
static bool BME688_isMeasuring(Sensor* sensor) {
    I2CSensor* i2c_sensor = (I2CSensor*)sensor->instance;
    return (bool)(unitemp_i2c_readReg(i2c_sensor, BME688_REG_STATUS) & 0x20);
}

bool unitemp_BME688_alloc(Sensor* sensor, char* args) {
    UNUSED(args);
    I2CSensor* i2c_sensor = (I2CSensor*)sensor->instance;
    BME688_instance* bme688_instance = malloc(sizeof(BME688_instance));
    if(bme688_instance == NULL) {
        FURI_LOG_E(APP_NAME, "Failed to allocation sensor %s instance", sensor->name);
        return false;
    }

    if(sensor->type == &BME688) bme688_instance->chip_id = BME688_ID;

    i2c_sensor->sensorInstance = bme688_instance;

    i2c_sensor->minI2CAdr = BME688_I2C_ADDR_MIN;
    i2c_sensor->maxI2CAdr = BME688_I2C_ADDR_MAX;
    return true;
}

bool unitemp_BME688_init(Sensor* sensor) {
    I2CSensor* i2c_sensor = (I2CSensor*)sensor->instance;
    //Перезагрузка
    unitemp_i2c_writeReg(i2c_sensor, 0xE0, 0xB6);
    //Чтение ID датчика
    uint8_t id = unitemp_i2c_readReg(i2c_sensor, 0xD0);
    if(id != BME688_ID) {
        FURI_LOG_E(
            APP_NAME,
            "Sensor %s returned wrong ID 0x%02X, expected 0x%02X",
            sensor->name,
            id,
            BME688_ID);
        return false;
    }

    unitemp_i2c_writeReg(
        i2c_sensor,
        BME688_REG_CTRL_HUM,
        (unitemp_i2c_readReg(i2c_sensor, BME688_REG_CTRL_HUM) & ~7) | BME688_HUM_OVERSAMPLING_1);
    unitemp_i2c_writeReg(
        i2c_sensor,
        BME688_REG_CTRL_MEAS,
        BME688_TEMP_OVERSAMPLING_2 | BME688_PRESS_OVERSAMPLING_4 | BME688_MODE_FORCED);
    //Настройка периода опроса и фильтрации значений
    unitemp_i2c_writeReg(
        i2c_sensor, BME688_REG_CONFIG, BME688_FILTER_COEFF_16 | BME688_SPI_3W_DISABLE);
    //Чтение калибровочных значений
    if(!BME688_readCalValues(i2c_sensor)) {
        FURI_LOG_E(APP_NAME, "Failed to read calibration values sensor %s", sensor->name);
        return false;
    }
    return true;
}

bool unitemp_BME688_deinit(Sensor* sensor) {
    I2CSensor* i2c_sensor = (I2CSensor*)sensor->instance;
    //Перевод в сон
    unitemp_i2c_writeReg(i2c_sensor, BME688_REG_CTRL_MEAS, BME688_MODE_SLEEP);
    return true;
}

UnitempStatus unitemp_BME688_update(Sensor* sensor) {
    I2CSensor* i2c_sensor = (I2CSensor*)sensor->instance;
    BME688_instance* instance = i2c_sensor->sensorInstance;

    uint32_t t = furi_get_tick();

    uint8_t buff[3];
    //Проверка инициализированности датчика
    unitemp_i2c_readRegArray(i2c_sensor, 0xF4, 2, buff);
    if(buff[0] == 0) {
        FURI_LOG_W(APP_NAME, "Sensor %s is not initialized!", sensor->name);
        return UT_SENSORSTATUS_ERROR;
    }

    unitemp_i2c_writeReg(
        i2c_sensor,
        BME688_REG_CTRL_MEAS,
        unitemp_i2c_readReg(i2c_sensor, BME688_REG_CTRL_MEAS) | 1);

    while(BME688_isMeasuring(sensor)) {
        if(furi_get_tick() - t > 100) {
            return UT_SENSORSTATUS_TIMEOUT;
        }
    }

    if(furi_get_tick() - instance->last_cal_update_time > BOSCH_CAL_UPDATE_INTERVAL) {
        BME688_readCalValues(i2c_sensor);
    }

    if(!unitemp_i2c_readRegArray(i2c_sensor, 0x1F, 3, buff)) return UT_SENSORSTATUS_TIMEOUT;
    int32_t adc_P = ((int32_t)buff[0] << 12) | ((int32_t)buff[1] << 4) | ((int32_t)buff[2] >> 4);
    if(!unitemp_i2c_readRegArray(i2c_sensor, 0x22, 3, buff)) return UT_SENSORSTATUS_TIMEOUT;
    int32_t adc_T = ((int32_t)buff[0] << 12) | ((int32_t)buff[1] << 4) | ((int32_t)buff[2] >> 4);
    if(!unitemp_i2c_readRegArray(i2c_sensor, 0x25, 2, buff)) return UT_SENSORSTATUS_TIMEOUT;
    int32_t adc_H = ((uint16_t)buff[0] << 8) | buff[1];

    sensor->temp = BME688_compensate_temperature(i2c_sensor, adc_T);
    sensor->pressure = BME688_compensate_pressure(i2c_sensor, adc_P);
    sensor->hum = BME688_compensate_humidity(i2c_sensor, adc_H);

    return UT_SENSORSTATUS_OK;
}

bool unitemp_BME688_free(Sensor* sensor) {
    I2CSensor* i2c_sensor = (I2CSensor*)sensor->instance;
    free(i2c_sensor->sensorInstance);
    return true;
}

// static VariableItemList* variable_item_list;
/**
 * @brief Функция обработки нажатия кнопки "Назад"
 *
 * @param context Указатель на данные приложения
 * @return ID вида в который нужно переключиться
 */
//static uint32_t _exit_callback(void* context) {
//    UNUSED(context);
//    variable_item_list_free(variable_item_list);
//
//    // TODO: maybe change to sensor actions
//    return UnitempViewGeneral;
//}

#define VIEW_ID UnitempViewSensorSpecificActions

//static void _enter_callback(void* context, uint32_t index) {
//    return;
//    UNUSED(context);
//    switch (index) {
//    case 0:
//        unitemp_popup(&I_Cry_dolph_55x52, "Result", "Here\nand here", VIEW_ID);
//        return;
//    }
//    variable_item_list_free(variable_item_list);
//}

bool unitemp_BME688_display_actions(Sensor* sensor) {
    UNUSED(sensor);
//    static View* view;
//    variable_item_list = variable_item_list_alloc();
//    variable_item_list_reset(variable_item_list);
//
//
//    FURI_LOG_I(APP_NAME, "3");
//
//    variable_item_list_add(variable_item_list, "Read Value", 1, NULL, NULL);
//    variable_item_list_add(variable_item_list, "Write Value", 1, NULL, NULL);
//
//
//    FURI_LOG_I(APP_NAME, "5");
//
//    //Добавление колбека на нажатие средней кнопки
//    variable_item_list_set_enter_callback(variable_item_list, _enter_callback, app);
//    FURI_LOG_I(APP_NAME, "6");
//    //Создание вида из списка
//    view = variable_item_list_get_view(variable_item_list);
//    FURI_LOG_I(APP_NAME, "7");
//    //Добавление колбека на нажатие кнопки "Назад"
//    view_set_previous_callback(view, _exit_callback);
//    FURI_LOG_I(APP_NAME, "7: %d", VIEW_ID);
//
//    //Добавление вида в диспетчер
//    view_dispatcher_add_view(app->view_dispatcher, VIEW_ID, view);
//    FURI_LOG_I(APP_NAME,"9");
    return true;
}