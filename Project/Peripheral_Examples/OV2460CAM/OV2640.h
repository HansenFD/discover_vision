/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV2640 driver.
 *
 */
#ifndef __OV2640_H__
#define __OV2640_H__
#include "stm32f4_discovery.h"

struct sensor_id {
    uint8_t MIDH;
    uint8_t MIDL;
    uint8_t PID;
    uint8_t VER;
};

enum sensor_pixformat {
    PIXFORMAT_RGB565,    /* 2BPP/RGB565*/
    PIXFORMAT_YUV422,    /* 2BPP/YUV422*/
    PIXFORMAT_GRAYSCALE, /* 1BPP/GRAYSCALE*/
    PIXFORMAT_JPEG,      /* JPEG/COMPRESSED */
};

enum sensor_framesize {
    FRAMESIZE_QQCIF,    /* 88x72     */
    FRAMESIZE_QQVGA,    /* 160x120   */
    FRAMESIZE_QCIF,     /* 176x144   */
    FRAMESIZE_QVGA,     /* 320x240   */
    FRAMESIZE_CIF,      /* 352x288   */
    FRAMESIZE_VGA,      /* 640x480   */
    FRAMESIZE_SVGA,     /* 800x600   */
    FRAMESIZE_SXGA,     /* 1280x1024 */
		FRAMESIZE_UXGA, 		/*UXGA*/
};

enum sensor_framerate {
    FRAMERATE_2FPS =0x9F,
    FRAMERATE_8FPS =0x87,
    FRAMERATE_15FPS=0x83,
    FRAMERATE_30FPS=0x81,
    FRAMERATE_60FPS=0x80,
};

enum sensor_gainceiling {
    GAINCEILING_2X,
    GAINCEILING_4X,
    GAINCEILING_8X,
    GAINCEILING_16X,
    GAINCEILING_32X,
    GAINCEILING_64X,
    GAINCEILING_128X,
};

enum sensor_attr {
    ATTR_CONTRAST=0,
    ATTR_BRIGHTNESS,
    ATTR_SATURATION,
    ATTR_GAINCEILING,
};

enum reset_polarity {
    ACTIVE_LOW,
    ACTIVE_HIGH
};

//extern const int res_width[];
//extern const int res_height[];

struct sensor_dev {
    struct sensor_id id;
    uint32_t vsync_pol;
    uint32_t hsync_pol;
    uint32_t pixck_pol;
    enum reset_polarity reset_pol;
    enum sensor_pixformat pixformat;
    enum sensor_framesize framesize;
    enum sensor_framerate framerate;
    enum sensor_gainceiling gainceiling;
    /* Sensor function pointers */
    int  (*reset)           ();
    int  (*set_pixformat)   (enum sensor_pixformat pixformat);
    int  (*set_framesize)   (enum sensor_framesize framesize);
    int  (*set_framerate)   (enum sensor_framerate framerate);
    int  (*set_contrast)    (int level);
    int  (*set_brightness)  (int level);
    int  (*set_saturation)  (int level);
    int  (*set_exposure)    (int exposure);
    int  (*set_gainceiling) (enum sensor_gainceiling gainceiling);
    int  (*set_quality)     (int quality);
};

int ov2640_init(struct sensor_dev *sensor);
#endif // __OV2640_H__
