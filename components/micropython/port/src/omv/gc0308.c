/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS},
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include "gc0308.h"
#include "dvp.h"
#include "plic.h"
#include "sleep.h"
#include "sensor.h"
#include "mphalport.h"
#include "cambus.h"
#include "printf.h"

static const uint8_t sensor_default_regs[][2] = {
    {0xFE, 0x00},
    {0xEC, 0x20},
    {0x05, 0x00},
    {0x06, 0x00},
    {0x07, 0x00},
    {0x08, 0x00},
    {0x09, 0x01},
    {0x0A, 0xE8},
    {0x0B, 0x02},
    {0x0C, 0x88},
    {0x0D, 0x02},
    {0x0E, 0x02},
    {0x10, 0x26},
    {0x11, 0x0D},
    {0x12, 0x2A},
    {0x13, 0x00},
    {0x14, 0x11},
    {0x15, 0x0A},
    {0x16, 0x05},
    {0x17, 0x01},
    {0x18, 0x44},
    {0x19, 0x44},
    {0x1A, 0x2A},
    {0x1B, 0x00},
    {0x1C, 0x49},
    {0x1D, 0x9A},
    {0x1E, 0x61},
    {0x1F, 0x00},
    {0x20, 0x7F},
    {0x21, 0xFA},
    {0x22, 0x57},
    {0x24, 0xA6},
    {0x25, 0x0F},
    {0x26, 0x03},
    {0x28, 0x00},
    {0x2D, 0x0A},
    {0x2F, 0x01},
    {0x30, 0xF7},
    {0x31, 0x50},
    {0x32, 0x00},
    {0x33, 0x28},
    {0x34, 0x2A},
    {0x35, 0x28},
    {0x39, 0x04},
    {0x3A, 0x20},
    {0x3B, 0x20},
    {0x3C, 0x00},
    {0x3D, 0x00},
    {0x3E, 0x00},
    {0x3F, 0x00},
    {0x50, 0x14},
    {0x52, 0x41},
    {0x53, 0x80},
    {0x54, 0x80},
    {0x55, 0x80},
    {0x56, 0x80},
    {0x8B, 0x20},
    {0x8C, 0x20},
    {0x8D, 0x20},
    {0x8E, 0x14},
    {0x8F, 0x10},
    {0x90, 0x14},
    {0x91, 0x3C},
    {0x92, 0x50},
    {0x5D, 0x12},
    {0x5E, 0x1A},
    {0x5F, 0x24},
    {0x60, 0x07},
    {0x61, 0x15},
    {0x62, 0x08},
    {0x64, 0x03},
    {0x66, 0xE8},
    {0x67, 0x86},
    {0x68, 0x82},
    {0x69, 0x18},
    {0x6A, 0x0F},
    {0x6B, 0x00},
    {0x6C, 0x5F},
    {0x6D, 0x8F},
    {0x6E, 0x55},
    {0x6F, 0x38},
    {0x70, 0x15},
    {0x71, 0x33},
    {0x72, 0xDC},
    {0x73, 0x00},
    {0x74, 0x02},
    {0x75, 0x3F},
    {0x76, 0x02},
    {0x77, 0x38},
    {0x78, 0x88},
    {0x79, 0x81},
    {0x7A, 0x81},
    {0x7B, 0x22},
    {0x7C, 0xFF},
    {0x93, 0x48},
    {0x94, 0x02},
    {0x95, 0x07},
    {0x96, 0xE0},
    {0x97, 0x40},
    {0x98, 0xF0},
    {0xB1, 0x40},
    {0xB2, 0x40},
    {0xB3, 0x40},
    {0xB6, 0xE0},
    {0xBD, 0x38},
    {0xBE, 0x36},
    {0xD0, 0xCB},
    {0xD1, 0x10},
    {0xD2, 0x90},
    {0xD3, 0x48},
    {0xD5, 0xF2},
    {0xD6, 0x16},
    {0xDB, 0x92},
    {0xDC, 0xA5},
    {0xDF, 0x23},
    {0xD9, 0x00},
    {0xDA, 0x00},
    {0xE0, 0x09},
    {0xED, 0x04},
    {0xEE, 0xA0},
    {0xEF, 0x40},
    {0x80, 0x03},
    {0x9F, 0x10},
    {0xA0, 0x20},
    {0xA1, 0x38},
    {0xA2, 0x4E},
    {0xA3, 0x63},
    {0xA4, 0x76},
    {0xA5, 0x87},
    {0xA6, 0xA2},
    {0xA7, 0xB8},
    {0xA8, 0xCA},
    {0xA9, 0xD8},
    {0xAA, 0xE3},
    {0xAB, 0xEB},
    {0xAC, 0xF0},
    {0xAD, 0xF8},
    {0xAE, 0xFD},
    {0xAF, 0xFF},
    {0xC0, 0x00},
    {0xC1, 0x10},
    {0xC2, 0x1C},
    {0xC3, 0x30},
    {0xC4, 0x43},
    {0xC5, 0x54},
    {0xC6, 0x65},
    {0xC7, 0x75},
    {0xC8, 0x93},
    {0xC9, 0xB0},
    {0xCA, 0xCB},
    {0xCB, 0xE6},
    {0xCC, 0xFF},
    {0xF0, 0x02},
    {0xF1, 0x01},
    {0xF2, 0x02},
    {0xF3, 0x30},
    {0xF7, 0x04},
    {0xF8, 0x02},
    {0xF9, 0x9F},
    {0xFA, 0x78},
    {0xFE, 0x01},
    {0x00, 0xF5},
    {0x02, 0x20},
    {0x04, 0x10},
    {0x05, 0x08},
    {0x06, 0x20},
    {0x08, 0x0A},
    {0x0A, 0xA0},
    {0x0B, 0x60},
    {0x0C, 0x08},
    {0x0E, 0x44},
    {0x0F, 0x32},
    {0x10, 0x41},
    {0x11, 0x37},
    {0x12, 0x22},
    {0x13, 0x19},
    {0x14, 0x44},
    {0x15, 0x44},
    {0x16, 0xC2},
    {0x17, 0xA8},
    {0x18, 0x18},
    {0x19, 0x50},
    {0x1A, 0xD8},
    {0x1B, 0xF5},
    {0x70, 0x40},
    {0x71, 0x58},
    {0x72, 0x30},
    {0x73, 0x48},
    {0x74, 0x20},
    {0x75, 0x60},
    {0x77, 0x20},
    {0x78, 0x32},
    {0x30, 0x03},
    {0x31, 0x40},
    {0x32, 0x10},
    {0x33, 0xE0},
    {0x34, 0xE0},
    {0x35, 0x00},
    {0x36, 0x80},
    {0x37, 0x00},
    {0x38, 0x04},
    {0x39, 0x09},
    {0x3A, 0x12},
    {0x3B, 0x1C},
    {0x3C, 0x28},
    {0x3D, 0x31},
    {0x3E, 0x44},
    {0x3F, 0x57},
    {0x40, 0x6C},
    {0x41, 0x81},
    {0x42, 0x94},
    {0x43, 0xA7},
    {0x44, 0xB8},
    {0x45, 0xD6},
    {0x46, 0xEE},
    {0x47, 0x0D},
    {0x62, 0xF7},
    {0x63, 0x68},
    {0x64, 0xD3},
    {0x65, 0xD3},
    {0x66, 0x60},
    {0xFE, 0x00},
    {0x01, 0x32},
    {0x02, 0x0C},
    {0x0F, 0x01},
    {0xE2, 0x00},
    {0xE3, 0x78},
    {0xE4, 0x00},
    {0xE5, 0xFE},
    {0xE6, 0x01},
    {0xE7, 0xE0},
    {0xE8, 0x01},
    {0xE9, 0xE0},
    {0xEA, 0x01},
    {0xEB, 0xE0},
    {0xFE, 0x00},
    {0x00, 0x00}
};

static int gc0308_read_reg(sensor_t *sensor, uint8_t reg_addr)
{
    uint8_t reg_data;

    if (cambus_readb(GC0308_ADDR, reg_addr, &reg_data) != 0)
    {
        return -1;
    }
    return reg_data;
}

static int gc0308_write_reg(sensor_t *sensor, uint8_t reg_addr, uint16_t reg_data)
{
    return cambus_writeb(GC0308_ADDR, reg_addr, (uint8_t)reg_data);
}

static int gc0308_set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    return 0;
}

static int gc0308_set_framesize(sensor_t *sensor, framesize_t framesize)
{
    struct subsample_cfg {
        uint16_t ratio_numerator;
        uint16_t ratio_denominator;
        uint8_t reg54;
        uint8_t reg56;
        uint8_t reg57;
        uint8_t reg58;
        uint8_t reg59;
    };
    const struct subsample_cfg subsample_cfgs[] = {
        { 84, 420, 0x55, 0x00, 0x00, 0x00, 0x00},
        {105, 420, 0x44, 0x00, 0x00, 0x00, 0x00},
        {140, 420, 0x33, 0x00, 0x00, 0x00, 0x00},
        {210, 420, 0x22, 0x00, 0x00, 0x00, 0x00},
        {240, 420, 0x77, 0x02, 0x46, 0x02, 0x46},
        {252, 420, 0x55, 0x02, 0x04, 0x02, 0x04},
        {280, 420, 0x33, 0x02, 0x00, 0x02, 0x00},
        {420, 420, 0x11, 0x00, 0x00, 0x00, 0x00},
    };
    const struct subsample_cfg *cfg = NULL;

    uint16_t win_w = resolution[framesize][0];
    uint16_t win_h = resolution[framesize][1];
    uint16_t col_s = (640 - win_w) >> 1;
    uint16_t row_s = (480 - win_h) >> 1;
    uint8_t i;

    for (i = 0; i < (sizeof(subsample_cfgs) / sizeof(subsample_cfgs[0])); i++)
    {
        cfg = &subsample_cfgs[i];
        if ((640 * cfg->ratio_numerator / cfg->ratio_denominator >= resolution[framesize][0]) && (480 * cfg->ratio_numerator / cfg->ratio_denominator >= resolution[framesize][1]))
        {
            win_w = resolution[framesize][0] * cfg->ratio_denominator / cfg->ratio_numerator;
            win_h = resolution[framesize][1] * cfg->ratio_denominator / cfg->ratio_numerator;
            col_s = (640 - win_w) >> 1;
            row_s = (480 - win_h) >> 1;
            break;
        }
    }

    cambus_writeb(GC0308_ADDR, 0xFE, 0x00);
    cambus_writeb(GC0308_ADDR, 0x05, (row_s >> 8) & 0xFF);
    cambus_writeb(GC0308_ADDR, 0x06, row_s & 0xFF);
    cambus_writeb(GC0308_ADDR, 0x07, (col_s >> 8) & 0xFF);
    cambus_writeb(GC0308_ADDR, 0x08, col_s & 0xFF);
    cambus_writeb(GC0308_ADDR, 0x09, ((win_h + 8) >> 8) & 0xFF);
    cambus_writeb(GC0308_ADDR, 0x0A, (win_h + 8) & 0xFF);
    cambus_writeb(GC0308_ADDR, 0x0B, ((win_w + 8) >> 8) & 0xFF);
    cambus_writeb(GC0308_ADDR, 0x0C, (win_w + 8) & 0xFF);
    cambus_writeb(GC0308_ADDR, 0xFE, 0x01);
    cambus_writeb(GC0308_ADDR, 0x54, cfg->reg54);
    cambus_writeb(GC0308_ADDR, 0x56, cfg->reg56);
    cambus_writeb(GC0308_ADDR, 0x57, cfg->reg57);
    cambus_writeb(GC0308_ADDR, 0x58, cfg->reg58);
    cambus_writeb(GC0308_ADDR, 0x59, cfg->reg59);
    cambus_writeb(GC0308_ADDR, 0xFE, 0x00);

    mp_hal_delay_ms(30);

    dvp_set_image_size(resolution[framesize][0], resolution[framesize][1]);

    return 0;
}

static int gc0308_set_framerate(sensor_t *sensor, framerate_t framerate)
{
    return 0;
}

#define NUM_CONTRAST_LEVELS (5)
static uint8_t contrast_regs[NUM_CONTRAST_LEVELS][1] = {
    {0x00},
    {0x20},
    {0x40},
    {0x60},
    {0x80}
};
static int gc0308_set_contrast(sensor_t *sensor, int level)
{
    level += (NUM_CONTRAST_LEVELS / 2);
    if (level < 0 || level > NUM_CONTRAST_LEVELS) {
        return -1;
    }

    cambus_writeb(GC0308_ADDR, 0xFE, 0x00);
    cambus_writeb(GC0308_ADDR, 0xB3, contrast_regs[level][0]);

    return 0;
}

static int gc0308_set_brightness(sensor_t *sensor, int level)
{
    return 0;
}

static int gc0308_set_saturation(sensor_t *sensor, int level)
{
    return 0;
}

static int gc0308_set_gainceiling(sensor_t *sensor, gainceiling_t gainceiling)
{
    return 0;
}

static int gc0308_set_quality(sensor_t *sensor, int qs)
{
    return 0;
}

static int gc0308_set_colorbar(sensor_t *sensor, int enable)
{
    uint8_t reg_data;

    cambus_writeb(GC0308_ADDR, 0xFE, 0x00);
    cambus_readb(GC0308_ADDR, 0x2E, &reg_data);
    reg_data &= ~(0x01 << 0);
    if (enable)
    {
        reg_data |= (0x01 << 0);
    }
    cambus_writeb(GC0308_ADDR, 0x2E, reg_data);
    return 0;
}

static int gc0308_set_auto_gain(sensor_t *sensor, int enable, float gain_db, float gain_db_ceiling)
{
    return 0;
}

static int gc0308_get_gain_db(sensor_t *sensor, float *gain_db)
{
    return 0;
}

static int gc0308_set_auto_exposure(sensor_t *sensor, int enable, int exposure_us)
{
    return 0;
}

static int gc0308_get_exposure_us(sensor_t *sensor, int *exposure_us)
{
    return 0;
}

static int gc0308_set_auto_whitebal(sensor_t *sensor, int enable, float r_gain_db, float g_gain_db, float b_gain_db)
{
    return 0;
}

static int gc0308_get_rgb_gain_db(sensor_t *sensor, float *r_gain_db, float *g_gain_db, float *b_gain_db)
{
    return 0;
}

static int gc0308_set_hmirror(sensor_t *sensor, int enable)
{
    uint8_t data;

    cambus_writeb(GC0308_ADDR, 0xFE, 0x00);
    cambus_readb(GC0308_ADDR, 0x14, &data);
    data &= ~(1 << 0);
    if (enable != 0)
    {
        data |= (1 << 0);
    }
    return cambus_writeb(GC0308_ADDR, 0x14, data);
}

static int gc0308_set_vflip(sensor_t *sensor, int enable)
{
    uint8_t data;

    cambus_writeb(GC0308_ADDR, 0xFE, 0x00);
    cambus_readb(GC0308_ADDR, 0x14, &data);
    data &= ~(1 << 1);
    if (enable != 0)
    {
        data |= (1 << 1);
    }
    return cambus_writeb(GC0308_ADDR, 0x14, data);
}

int gc0308_reset(sensor_t *sensor)
{
    uint16_t index;

    cambus_writeb(GC0308_ADDR, 0xFE, 0xF0);
    mp_hal_delay_ms(80);

    for (index=0; index<(sizeof(sensor_default_regs)/sizeof(sensor_default_regs[0])); index++)
    {
        cambus_writeb(GC0308_ADDR, sensor_default_regs[index][0], sensor_default_regs[index][1]);
    }

    mp_hal_delay_ms(80);
    cambus_writeb(GC0308_ADDR, 0xFE, 0x00);

    return 0;
}

int gc0308_set_windowing(framesize_t fraemsize, int x, int y, int w, int h)
{
    if (fraemsize == FRAMESIZE_QVGA)
    {
        cambus_writeb(GC0308_ADDR, 0xFE, 0x00);
        cambus_writeb(GC0308_ADDR, 0x05, (y >> 8) & 0xFF);
        cambus_writeb(GC0308_ADDR, 0x06, y & 0xFF);
        cambus_writeb(GC0308_ADDR, 0x07, (x >> 8) & 0xFF);
        cambus_writeb(GC0308_ADDR, 0x08, x & 0xFF);
        cambus_writeb(GC0308_ADDR, 0x09, (h >> 8) & 0xFF);
        cambus_writeb(GC0308_ADDR, 0x0A, h & 0xFF);
        cambus_writeb(GC0308_ADDR, 0x0B, (w >> 8) & 0xFF);
        cambus_writeb(GC0308_ADDR, 0x0C, w & 0xFF);
    }

    return 0;
}

int gc0308_init(sensor_t *sensor)
{
    //Initialize sensor structure.
    sensor->gs_bpp              = 2;
    sensor->reset               = gc0308_reset;
    sensor->read_reg            = gc0308_read_reg;
    sensor->write_reg           = gc0308_write_reg;
    sensor->set_pixformat       = gc0308_set_pixformat;
    sensor->set_framesize       = gc0308_set_framesize;
    sensor->set_framerate       = gc0308_set_framerate;
    sensor->set_contrast        = gc0308_set_contrast;
    sensor->set_brightness      = gc0308_set_brightness;
    sensor->set_saturation      = gc0308_set_saturation;
    sensor->set_gainceiling     = gc0308_set_gainceiling;
    sensor->set_quality         = gc0308_set_quality;
    sensor->set_colorbar        = gc0308_set_colorbar;
    sensor->set_auto_gain       = gc0308_set_auto_gain;
    sensor->get_gain_db         = gc0308_get_gain_db;
    sensor->set_auto_exposure   = gc0308_set_auto_exposure;
    sensor->get_exposure_us     = gc0308_get_exposure_us;
    sensor->set_auto_whitebal   = gc0308_set_auto_whitebal;
    sensor->get_rgb_gain_db     = gc0308_get_rgb_gain_db;
    sensor->set_hmirror         = gc0308_set_hmirror;
    sensor->set_vflip           = gc0308_set_vflip;
	sensor->set_windowing       = gc0308_set_windowing;

    // Set sensor flags
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_VSYNC, 0);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_HSYNC, 0);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_PIXCK, 1);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_FSYNC, 0);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_JPEGE, 1);

    return 0;
}
