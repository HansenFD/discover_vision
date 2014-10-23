/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV2640 driver.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sccb.h"
#include "ov2640.h"
#include "ov2640_regs.h"

#define SVGA_HSIZE     (800)
#define SVGA_VSIZE     (600)

#define UXGA_HSIZE     (1600)
#define UXGA_VSIZE     (1200)

//Gain = (((reg0x00 & 0xf0)>>4) + 1)*(1 + (reg0x00 & 0x0f)/16)

#define GAIN_1X 	0X00
#define GAIN_2X		0X10
#define GAIN_4X		0X30
#define GAIN_8X		0X70
#define GAIN_16X 	0XF0
#define GAIN_32X	0XFF

#define AUTO_EXPOSURE_ENABLED_AUTO_GAIN_ENABLED 	 1
//#define AUTO_EXPOSURE_DISABLED_AUTO_GAIN_ENABLED	 1
//#define AUTO_EXPOSURE_ENABLED_AUTO_GAIN_DISABLED	 1
//#define AUTO_EXPOSURE_DISABLED_AUTO_GAIN_DISABLED	 1	

				
unsigned short shutsp;


#define LIGHT_MODE_AUTO   1
#define LIGHT_MODE_SUNNY  2
#define LIGHT_MODE_CLOUDY 3
#define LIGHT_MODE_OFFICE 4
#define LIGHT_MODE_HOME   5

#define SPECIAL_EFFECT_ANTIQUE	1
#define SPECIAL_EFFECT_BLUISH		2	
#define SPECIAL_EFFECT_GREENISH 3
#define SPECIAL_EFFECT_REDDISH	4
#define SPECIAL_EFFECT_BW				5
#define SPECIAL_EFFECT_NEGATIVE 6
#define SPECIAL_EFFECT_BWNEG		7
#define SPECIAL_EFFECT_NORMAL 	8

const static u8 OV2640_AUTOEXPOSURE_LEVEL[][2]=   
{   
    0xFF,   0x01,  
    0x24,   0x20,   
    0x25,   0x18,     
    0x26,   0x60,    
    0x00,   0x00, 
	
	  0xFF,   0x01,  
    0x24,   0x34,    
    0x25,   0x1c,   
    0x26,   0x70,     
    0x00,   0x00,  
	
	  0xFF,   0x01,     
    0x24,   0x3e,    
    0x25,   0x38,    
    0x26,   0x81,   
    0x00,   0x00, 

    0xFF,   0x01,    
    0x24,   0x48,  
    0x25,   0x40,    
    0x26,   0x81,     
    0x00,   0x00, 

    0xFF,   0x01,    
    0x24,   0x58,  
    0x25,   0x50,     
    0x26,   0x92,      
    0x00,   0x00,
	
};   
   




static const unsigned char test[][2]={
  {0xff, 0x01},
  {0x11, 0x01},
  {0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02Îª²ÊÌõ
  {0x17, 0x11}, // HREFST[10:3]
  {0x18, 0x75}, // HREFEND[10:3]
  {0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
  {0x19, 0x01}, // VSTRT[9:2]
  {0x1a, 0x97}, // VEND[9:2]
  {0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
  {0x37, 0x40},
  {0x4f, 0xbb},
  {0x50, 0x9c},
  {0x5a, 0x57},
  {0x6d, 0x80},
  {0x3d, 0x34},
  {0x39, 0x02},
  {0x35, 0x88},
  {0x22, 0x0a},
  {0x37, 0x40},
  {0x34, 0xa0},
  {0x06, 0x02},
  {0x0d, 0xb7},
  {0x0e, 0x01},
  
  ////////////////
  /*
  //176*144
   0xff,      0x00,
      0xc0,      0xC8,
      0xc1,      0x96,
      0x8c,      0x00,
      0x86,      0x3D,
      0x50,      0x9B,
      0x51,      0x90,
      0x52,      0x2C,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x5a,      0x2C,
      0x5b,      0x24,
      0x5c,      0x00,
      0xd3,      0x7F,
	  ////////////
	  */
/*	  
	 ////////////////
	 //320*240
	  0xff,      0x00,
      0xe0,      0x04,
      0xc0,      0xc8,
      0xc1,      0x96,
      0x86,      0x3d,
      0x50,      0x92,
      0x51,      0x90,
      0x52,      0x2c,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x57,      0x00,
      0x5a,      0x50,
      0x5b,      0x3c,
      0x5c,      0x00,
      0xd3,      0x7F,
      0xe0,      0x00,
	  ///////////////////
*/	  
 /*
0xff,      0x00,
      0xe0,      0x04,
      0xc0,      0xc8,
      0xc1,      0x96,
      0x86,      0x35,
      0x50,      0x92,
      0x51,      0x90,
      0x52,      0x2c,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x57,      0x00,
      0x5a,      0x58,
      0x5b,      0x48,
      0x5c,      0x00,
      0xd3,      0x08,
      0xe0,      0x00
*/
/*
//640*480	  
 	  0xff,      0x00,
      0xe0,      0x04,
      0xc0,      0xc8,
      0xc1,      0x96,
      0x86,      0x3d,
      0x50,      0x89,
      0x51,      0x90,
      0x52,      0x2c,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x57,      0x00,
      0x5a,      0xa0,
      0x5b,      0x78,
      0x5c,      0x00,
      0xd3,      0x04,
      0xe0,      0x00
*/	  
	  /////////////////////
	  /*
	  //800*600
	  0xff,      0x00,
      0xe0,      0x04,
      0xc0,      0xc8,
      0xc1,      0x96,
      0x86,      0x35,
      0x50,      0x89,
      0x51,      0x90,
      0x52,      0x2c,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x57,      0x00,
      0x5a,      0xc8,
      0x5b,      0x96,
      0x5c,      0x00,
      0xd3,      0x02,
      0xe0,      0x00,
	  
	  */
	  //1280*1024
	/*  
	  0xff,      0x00,
      0xe0,      0x04,
      0xc0,      0xc8,
      0xc1,      0x96,
      0x86,      0x3d,
      0x50,      0x00,
      0x51,      0x90,
      0x52,      0x2c,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x57,      0x00,
      0x5a,      0x40,
      0x5b,      0xf0,
      0x5c,      0x01,
      0xd3,      0x02,
      0xe0,      0x00
	 */ 
	  
	  /////////////////////
	  //1600*1200
	  
	  0xff,      0x00,
      0xe0,      0x04,
      0xc0,      0xc8,
      0xc1,      0x96,
      0x86,      0x3d,
      0x50,      0x00,
      0x51,      0x90,
      0x52,      0x2c,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x57,      0x00,
      0x5a,      0x90,
      0x5b,      0x2C,
      0x5c,      0x05,//bit2->1;bit[1:0]->1
      0xd3,      0x02,
      0xe0,      0x00,
	  /////////////////////
	  
	/*  
	  //1024*768
	   0xff,      0x00,
      0xc0,      0xC8,
      0xc1,      0x96,
      0x8c,      0x00,
      0x86,      0x3D,
      0x50,      0x00,
      0x51,      0x90,
      0x52,      0x2C,
      0x53,      0x00,
      0x54,      0x00,
      0x55,      0x88,
      0x5a,      0x00,
      0x5b,      0xC0,
      0x5c,      0x01,
      0xd3,      0x02
	 */ 
0,0
};
static const uint8_t default_regs[][2] = {
    { BANK_SEL, BANK_SEL_DSP },
    { 0x2c,     0xff },
    { 0x2e,     0xdf },
    { BANK_SEL, BANK_SEL_SENSOR },
    { 0x3c,     0x32 },
    { CLKRC,    0x80 }, /* Set PCLK divider */
    { COM2,     COM2_OUT_DRIVE_3x }, /* Output drive x2 */
#ifdef OPENMV2
    { REG04,    0xF8}, /* Mirror/VFLIP/AEC[1:0] */
#else
    { REG04_SET(REG04_HREF_EN)},
#endif

 // { COM8,     COM8_SET(COM8_BNDF_EN | COM8_AGC_EN | COM8_AEC_EN) },
#if AUTO_EXPOSURE_ENABLED_AUTO_GAIN_ENABLED 
 { COM8,     COM8_SET(COM8_BNDF_EN | COM8_AGC_EN | COM8_AEC_EN) },
#elif AUTO_EXPOSURE_DISABLED_AUTO_GAIN_ENABLED
 { COM8,     COM8_SET(COM8_BNDF_EN | COM8_AGC_EN) },
#elif	AUTO_EXPOSURE_ENABLED_AUTO_GAIN_DISABLED
 { COM8,     COM8_SET(COM8_BNDF_EN | COM8_AEC_EN) },
#elif AUTO_EXPOSURE_DISABLED_AUTO_GAIN_DISABLED
	{ COM8,     COM8_SET(COM8_BNDF_EN ) }, 	
#endif
    { COM9,     COM9_AGC_SET(COM9_AGC_GAIN_8x)},
    { 0x2c,     0x0c },
    { 0x33,     0x78 },
    { 0x3a,     0x33 },
    { 0x3b,     0xfb },
    { 0x3e,     0x00 },
    { 0x43,     0x11 },
    { 0x16,     0x10 },
    { 0x39,     0x02 },
    { 0x35,     0x88 },
    { 0x22,     0x0a },
    { 0x37,     0x40 },
    { 0x23,     0x00 },
    { ARCOM2,   0xa0 },
    { 0x06,     0x02 },
    { 0x06,     0x88 },
    { 0x07,     0xc0 },
    { 0x0d,     0xb7 },
    { 0x0e,     0x01 },
    { 0x4c,     0x00 },
    { 0x4a,     0x81 },
    { 0x21,     0x99 },
    { AEW,      0x40 },
    { AEB,      0x38 },
    /* AGC/AEC fast mode operating region */
    { VV,       VV_AGC_TH_SET(0x08, 0x02) },
    { COM19,    0x00 }, /* Zoom control 2 MSBs */
    { ZOOMS,    0x00 }, /* Zoom control 8 MSBs */
    { 0x5c,     0x00 },
    { 0x63,     0x00 },
    { FLL,      0x00 },
    { FLH,      0x00 },

    /* Set banding filter */
    { COM3,     COM3_BAND_SET(COM3_BAND_AUTO) },
    { REG5D,    0x55 },
    { REG5E,    0x7d },
    { REG5F,    0x7d },
    { REG60,    0x55 },
    { HISTO_LOW,   0x70 },
    { HISTO_HIGH,  0x80 },
    { 0x7c,     0x05 },
    { 0x20,     0x80 },
    { 0x28,     0x30 },
    { 0x6c,     0x00 },
    { 0x6d,     0x80 },
    { 0x6e,     0x00 },
    { 0x70,     0x02 },
    { 0x71,     0x94 },
    { 0x73,     0xc1 },
    { 0x3d,     0x34 },
    //{ COM7,   COM7_RES_UXGA | COM7_ZOOM_EN },
    { 0x5a,     0x57 },
    { BD50,     0xbb },
    { BD60,     0x9c },

    { BANK_SEL, BANK_SEL_DSP },
    { 0xe5,     0x7f },
    { MC_BIST,  MC_BIST_RESET | MC_BIST_BOOT_ROM_SEL },
    { 0x41,     0x24 },
    { RESETR,    RESET_JPEG | RESET_DVP },
    { 0x76,     0xff },
    { 0x33,     0xa0 },
    { 0x42,     0x20 },
    { 0x43,     0x18 },
    { 0x4c,     0x00 },
    { CTRL3,    CTRL3_BPC_EN | CTRL3_WPC_EN | 0x10 },
    { 0x88,     0x3f },
    { 0xd7,     0x03 },
    { 0xd9,     0x10 },
    { R_DVP_SP , R_DVP_SP_AUTO_MODE | 0x2 },
    { 0xc8,     0x08 },
    { 0xc9,     0x80 },
    { BPADDR,   0x00 },
    { BPDATA,   0x00 },
    { BPADDR,   0x03 },
    { BPDATA,   0x48 },
    { BPDATA,   0x48 },
    { BPADDR,   0x08 },
    { BPDATA,   0x20 },
    { BPDATA,   0x10 },
    { BPDATA,   0x0e },
    { 0x90,     0x00 },
    { 0x91,     0x0e },
    { 0x91,     0x1a },
    { 0x91,     0x31 },
    { 0x91,     0x5a },
    { 0x91,     0x69 },
    { 0x91,     0x75 },
    { 0x91,     0x7e },
    { 0x91,     0x88 },
    { 0x91,     0x8f },
    { 0x91,     0x96 },
    { 0x91,     0xa3 },
    { 0x91,     0xaf },
    { 0x91,     0xc4 },
    { 0x91,     0xd7 },
    { 0x91,     0xe8 },
    { 0x91,     0x20 },
    { 0x92,     0x00 },
    { 0x93,     0x06 },
    { 0x93,     0xe3 },
    { 0x93,     0x03 },
    { 0x93,     0x03 },
    { 0x93,     0x00 },
    { 0x93,     0x02 },
    { 0x93,     0x00 },
    { 0x93,     0x00 },
    { 0x93,     0x00 },
    { 0x93,     0x00 },
    { 0x93,     0x00 },
    { 0x93,     0x00 },
    { 0x93,     0x00 },
    { 0x96,     0x00 },
    { 0x97,     0x08 },
    { 0x97,     0x19 },
    { 0x97,     0x02 },
    { 0x97,     0x0c },
    { 0x97,     0x24 },
    { 0x97,     0x30 },
    { 0x97,     0x28 },
    { 0x97,     0x26 },
    { 0x97,     0x02 },
    { 0x97,     0x98 },
    { 0x97,     0x80 },
    { 0x97,     0x00 },
    { 0x97,     0x00 },
    { 0xa4,     0x00 },
    { 0xa8,     0x00 },
    { 0xc5,     0x11 },
    { 0xc6,     0x51 },
    { 0xbf,     0x80 },
    { 0xc7,     0x10 },
    { 0xb6,     0x66 },
    { 0xb8,     0xA5 },
    { 0xb7,     0x64 },
    { 0xb9,     0x7C },
    { 0xb3,     0xaf },
    { 0xb4,     0x97 },
    { 0xb5,     0xFF },
    { 0xb0,     0xC5 },
    { 0xb1,     0x94 },
    { 0xb2,     0x0f },
    { 0xc4,     0x5c },
    { 0xa6,     0x00 },
    { 0xa7,     0x20 },
    { 0xa7,     0xd8 },
    { 0xa7,     0x1b },
    { 0xa7,     0x31 },
    { 0xa7,     0x00 },
    { 0xa7,     0x18 },
    { 0xa7,     0x20 },
    { 0xa7,     0xd8 },
    { 0xa7,     0x19 },
    { 0xa7,     0x31 },
    { 0xa7,     0x00 },
    { 0xa7,     0x18 },
    { 0xa7,     0x20 },
    { 0xa7,     0xd8 },
    { 0xa7,     0x19 },
    { 0xa7,     0x31 },
    { 0xa7,     0x00 },
    { 0xa7,     0x18 },
    { 0x7f,     0x00 },
    { 0xe5,     0x1f },
    { 0xe1,     0x77 },
    { 0xdd,     0x7f },
    { CTRL0,    CTRL0_YUV422 | CTRL0_YUV_EN | CTRL0_RGB_EN },
    { 0x00,     0x00 }
};



const uint8_t svga_regs[][2] = {
        { BANK_SEL, BANK_SEL_SENSOR },
        /* DSP input image resoultion and window size control */
        { COM7,    COM7_RES_SVGA},
        { COM1,    0x0A }, /* UXGA=0x0F, SVGA=0x0A, CIF=0x06 */
        { REG32,   0x09 }, /* UXGA=0x36, SVGA/CIF=0x09 */

        { HSTART,  0x11 }, /* UXGA=0x11, SVGA/CIS=0x11 */
        { HSTOP,   0x43 }, /* UXGA=0x75, SVGA/CIF=0x43 */

        { VSTART,  0x00 }, /* UXGA=0x01, SVGA/CIF=0x00 */
        { VSTOP,   0x4b }, /* UXGA=0x97, SVGA/CIS=0x4b */
        { 0x3d,    0x38 }, /* UXGA=0x34, SVGA/CIF=0x38 */

        { 0x35,    0xda },
        { 0x22,    0x1a },
        { 0x37,    0xc3 },
        { 0x34,    0xc0 },
        { 0x06,    0x88 },
        { 0x0d,    0x87 },
        { 0x0e,    0x41 },
        { 0x42,    0x03 },

        /* Set DSP input image size and offset.
           The sensor output image can be scaled with OUTW/OUTH */
        { BANK_SEL, BANK_SEL_DSP },
        { R_BYPASS, R_BYPASS_DSP_BYPAS },

        { RESETR,   RESET_DVP },
        { HSIZE8,  (SVGA_HSIZE>>3)}, /* Image Horizontal Size HSIZE[10:3] */
        { VSIZE8,  (SVGA_VSIZE>>3)}, /* Image Vertiacl Size VSIZE[10:3] */

        /* {HSIZE[11], HSIZE[2:0], VSIZE[2:0]} */
        { SIZEL,   ((SVGA_HSIZE>>6)&0x40) | ((SVGA_HSIZE&0x7)<<3) | (SVGA_VSIZE&0x7)},

        { XOFFL,   0x00 }, /* OFFSET_X[7:0] */
        { YOFFL,   0x00 }, /* OFFSET_Y[7:0] */
        { HSIZE,   ((SVGA_HSIZE>>2)&0xFF) }, /* H_SIZE[7:0]= HSIZE/4 */
        { VSIZE,   ((SVGA_VSIZE>>2)&0xFF) }, /* V_SIZE[7:0]= VSIZE/4 */

        /* V_SIZE[8]/OFFSET_Y[10:8]/H_SIZE[8]/OFFSET_X[10:8] */
        { VHYX,    ((SVGA_VSIZE>>3)&0x80) | ((SVGA_HSIZE>>7)&0x08) },
        { TEST,    (SVGA_HSIZE>>4)&0x80}, /* H_SIZE[9] */

        { CTRL2,   CTRL2_DCW_EN | CTRL2_SDE_EN |
          CTRL2_UV_AVG_EN | CTRL2_CMX_EN | CTRL2_UV_ADJ_EN },

        /* H_DIVIDER/V_DIVIDER */
        { CTRLI,   CTRLI_LP_DP | 0x00},
        /* DVP prescalar */
        { R_DVP_SP, R_DVP_SP_AUTO_MODE},

        { R_BYPASS, R_BYPASS_DSP_EN },
        { RESETR,    0x00 },
        {0, 0},
};

static const uint8_t uxga_regs[][2] = {
        { BANK_SEL, BANK_SEL_SENSOR },
        /* DSP input image resoultion and window size control */
        { COM7,    COM7_RES_UXGA},
        { COM1,    0x0F }, /* UXGA=0x0F, SVGA=0x0A, CIF=0x06 */
        { REG32,   0x36 }, /* UXGA=0x36, SVGA/CIF=0x09 */

        { HSTART,  0x11 }, /* UXGA=0x11, SVGA/CIS=0x11 */
        { HSTOP,   0x75 }, /* UXGA=0x75, SVGA/CIF=0x43 */

        { VSTART,  0x01 }, /* UXGA=0x01, SVGA/CIF=0x00 */
        { VSTOP,   0x97 }, /* UXGA=0x97, SVGA/CIS=0x4b */
        { 0x3d,    0x34 }, /* UXGA=0x34, SVGA/CIF=0x38 */

        { 0x35,    0x88 },
        { 0x22,    0x0a },
        { 0x37,    0x40 },
        { 0x34,    0xa0 },
        { 0x06,    0x02 },
        { 0x0d,    0xb7 },
        { 0x0e,    0x01 },
        { 0x42,    0x83 },

        /* Set DSP input image size and offset.
           The sensor output image can be scaled with OUTW/OUTH */
        { BANK_SEL, BANK_SEL_DSP },
        { R_BYPASS, R_BYPASS_DSP_BYPAS },

        { RESETR,   RESET_DVP },
        { HSIZE8,  (UXGA_HSIZE>>3)}, /* Image Horizontal Size HSIZE[10:3] */
        { VSIZE8,  (UXGA_VSIZE>>3)}, /* Image Vertiacl Size VSIZE[10:3] */

        /* {HSIZE[11], HSIZE[2:0], VSIZE[2:0]} */
        { SIZEL,   ((UXGA_HSIZE>>6)&0x40) | ((UXGA_HSIZE&0x7)<<3) | (UXGA_VSIZE&0x7)},

        { XOFFL,   0x00 }, /* OFFSET_X[7:0] */
        { YOFFL,   0x00 }, /* OFFSET_Y[7:0] */
        { HSIZE,   ((UXGA_HSIZE>>2)&0xFF) }, /* H_SIZE[7:0] real/4 */
        { VSIZE,   ((UXGA_VSIZE>>2)&0xFF) }, /* V_SIZE[7:0] real/4 */

        /* V_SIZE[8]/OFFSET_Y[10:8]/H_SIZE[8]/OFFSET_X[10:8] */
        { VHYX,    ((UXGA_VSIZE>>3)&0x80) | ((UXGA_HSIZE>>7)&0x08) },
        { TEST,    (UXGA_HSIZE>>4)&0x80}, /* H_SIZE[9] */

        { CTRL2,   CTRL2_DCW_EN | CTRL2_SDE_EN |
            CTRL2_UV_AVG_EN | CTRL2_CMX_EN | CTRL2_UV_ADJ_EN },

        /* H_DIVIDER/V_DIVIDER */
        { CTRLI,   CTRLI_LP_DP | 0x00},
        /* DVP prescalar */
        { R_DVP_SP, R_DVP_SP_AUTO_MODE | 0x04},

        { R_BYPASS, R_BYPASS_DSP_EN },
        { RESETR,    0x00 },
        {0, 0},
};

static const uint8_t yuv422_regs[][2] = {
        { BANK_SEL, BANK_SEL_DSP },
        { RESETR,   RESET_DVP},
        { IMAGE_MODE, IMAGE_MODE_YUV422 },
        { 0xD7,     0x01 },
        { 0xE1,     0x67 },
        { RESETR,    0x00 },
        {0, 0},
};

static const uint8_t rgb565_regs[][2] = {
        { BANK_SEL, BANK_SEL_DSP },
        { RESETR,   RESET_DVP},
        { IMAGE_MODE, IMAGE_MODE_RGB565 },
        { 0xD7,     0x03 },
        { 0xE1,     0x77 },
        { RESETR,    0x00 },
        {0, 0},
};

static const uint8_t jpeg_regs[][2] = {
        { BANK_SEL, BANK_SEL_DSP },
        { RESETR,   RESET_DVP},
        { IMAGE_MODE, IMAGE_MODE_JPEG_EN|IMAGE_MODE_RGB565 },
        { 0xD7,     0x03 },
        { 0xE1,     0x77 },
        { QS,       0x0C },
        { RESETR,    0x00 },
        {0, 0},
};

#define NUM_BRIGHTNESS_LEVELS (5)
static const uint8_t brightness_regs[NUM_BRIGHTNESS_LEVELS + 1][5] = {
    { BPADDR, BPDATA, BPADDR, BPDATA, BPDATA },
    { 0x00, 0x04, 0x09, 0x00, 0x00 }, /* -2 */
    { 0x00, 0x04, 0x09, 0x10, 0x00 }, /* -1 */
    { 0x00, 0x04, 0x09, 0x20, 0x00 }, /*  0 */
    { 0x00, 0x04, 0x09, 0x30, 0x00 }, /* +1 */
    { 0x00, 0x04, 0x09, 0x40, 0x00 }, /* +2 */
};

#define NUM_CONTRAST_LEVELS (5)
static const uint8_t contrast_regs[NUM_CONTRAST_LEVELS + 1][7] = {
    { BPADDR, BPDATA, BPADDR, BPDATA, BPDATA, BPDATA, BPDATA },
    { 0x00, 0x04, 0x07, 0x20, 0x18, 0x34, 0x06 }, /* -2 */
    { 0x00, 0x04, 0x07, 0x20, 0x1c, 0x2a, 0x06 }, /* -1 */
    { 0x00, 0x04, 0x07, 0x20, 0x20, 0x20, 0x06 }, /*  0 */
    { 0x00, 0x04, 0x07, 0x20, 0x24, 0x16, 0x06 }, /* +1 */
    { 0x00, 0x04, 0x07, 0x20, 0x28, 0x0c, 0x06 }, /* +2 */
};

#define NUM_SATURATION_LEVELS (5)
static const uint8_t saturation_regs[NUM_SATURATION_LEVELS + 1][5] = {
    { BPADDR, BPDATA, BPADDR, BPDATA, BPDATA },
    { 0x00, 0x02, 0x03, 0x28, 0x28 }, /* -2 */
    { 0x00, 0x02, 0x03, 0x38, 0x38 }, /* -1 */
    { 0x00, 0x02, 0x03, 0x48, 0x48 }, /*  0 */
    { 0x00, 0x02, 0x03, 0x58, 0x58 }, /* +1 */
    { 0x00, 0x02, 0x03, 0x58, 0x58 }, /* +2 */
};

#define NUM_SPECIAL_EFFECTS (8)
static const uint8_t effects_regs[NUM_SPECIAL_EFFECTS + 1][5] = {
    { BPADDR, BPDATA, BPADDR, BPDATA, BPDATA },
    { 0x00, 0x18, 0x05, 0x40, 0xa6 }, /* Antique */
    { 0x00, 0x18, 0x05, 0xa0, 0x40 }, /* Bluish */
    { 0x00, 0x18, 0x05, 0x40, 0x40 }, /* Greenish */
    { 0x00, 0x18, 0x05, 0x40, 0xc0 }, /* Reddish */
    { 0x00, 0x18, 0x05, 0x80, 0x80 }, /* B&W */
		{ 0x00, 0x40, 0x05, 0x80, 0x80 }, /* Negative */
		{ 0x00, 0x58, 0x05, 0x80, 0x80 }, /* B&W Negative */
		{ 0x00, 0x00, 0x05, 0x80, 0x80 }, /* Normal */
};

const int res_width[] = {
    88,     /* QQCIF */
    160,    /* QQVGA */
    176,    /* QCIF  */
    320,    /* QVGA  */
    352,    /* CIF   */
    640,    /* VGA   */
    800,    /* SVGA  */
    1280,   /* SXGA  */
		1600,   /*UXGA*/
};

const int res_height[]= {
    72,     /* QQCIF */
    120,    /* QQVGA */
    144,    /* QCIF  */
    240,    /* QVGA  */
    288,    /* CIF   */
    480,    /* VGA   */
    600,    /* SVGA   */
    1024,   /* SXGA  */
	  1200,    /*UXGA*/
};

enum exposure_levels 
{   
    AUTOEXPOSURE_LEVEL0,   
    AUTOEXPOSURE_LEVEL1,   
    AUTOEXPOSURE_LEVEL2,   
    AUTOEXPOSURE_LEVEL3,   
    AUTOEXPOSURE_LEVEL4   
};   

extern void Delayms(unsigned iv);

void systick_sleep(unsigned delay)
{
	Delayms(delay);
}



static int reset()
{
    int i=0;
    const uint8_t (*regs)[2];

    /* Reset all registers */
    SCCB_Write(BANK_SEL, BANK_SEL_SENSOR);
    SCCB_Write(COM7, COM7_SRST);

    /* delay n ms */
    systick_sleep(10);

    i = 0;
    regs = default_regs;
    /* Write initial regsiters */
    while (regs[i][0]) {
        SCCB_Write(regs[i][0], regs[i][1]);
        i++;
    }

    i = 0;
    regs = svga_regs;
    /* Write DSP input regsiters */
    while (regs[i][0]) {
        SCCB_Write(regs[i][0], regs[i][1]);
        i++;
    }

    return 0;
}

static int set_pixformat(enum sensor_pixformat pixformat)
{
    int i=0;
    const uint8_t (*regs)[2]=NULL;

    /* read pixel format reg */
    switch (pixformat) {
        case PIXFORMAT_RGB565:
            regs = rgb565_regs;
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_GRAYSCALE:
            regs = yuv422_regs;
            break;
        case PIXFORMAT_JPEG:
            regs = jpeg_regs;
            break;
        default:
            return -1;
    }

    /* Write initial regsiters */
    while (regs[i][0]) {
        SCCB_Write(regs[i][0], regs[i][1]);
        i++;
    }
    return 0;
}
static int set_exposure(enum exposure_levels level)
{
	
	unsigned char i;
	const uint8_t (*regs)[2];
	
	
	if ( level > AUTOEXPOSURE_LEVEL4 )
		level = AUTOEXPOSURE_LEVEL4;
	
	//SET PIXEL FORMAT
		i = level*2;
		regs = OV2640_AUTOEXPOSURE_LEVEL;
		
		while (regs[i][0]) {
        SCCB_Write(regs[i][0], regs[i][1]);
        i++;
    }
	
		return 0;
}



static int set_framesize(enum sensor_framesize framesize)
{
    int ret=0;
    uint8_t clkrc = 0x80;
    uint16_t w=res_width[framesize];
    uint16_t h=res_height[framesize];

    if (framesize > FRAMESIZE_QVGA) {
        clkrc =0x81;
			  // clkrc =0x80;
    }

    /* Disable DSP */
    ret |= SCCB_Write(BANK_SEL, BANK_SEL_DSP);
    ret |= SCCB_Write(R_BYPASS, R_BYPASS_DSP_BYPAS);

    /* Write output width */
    ret |= SCCB_Write(ZMOW, (w>>2)&0xFF); /* OUTW[7:0] (real/4) */
    ret |= SCCB_Write(ZMOH, (h>>2)&0xFF); /* OUTH[7:0] (real/4) */
    ret |= SCCB_Write(ZMHH, ((h>>8)&0x04)|((w>>10)&0x03)); /* OUTH[8]/OUTW[9:8] */

    /* Set CLKRC */
    ret |= SCCB_Write(BANK_SEL, BANK_SEL_SENSOR);
    ret |= SCCB_Write(CLKRC, clkrc);


    /* Enable DSP */
    ret |= SCCB_Write(BANK_SEL, BANK_SEL_DSP);
    ret |= SCCB_Write(R_BYPASS, R_BYPASS_DSP_EN);
    return ret;
}

static int set_framerate(enum sensor_framerate framerate)
{
    return 0;
}

static int set_contrast(int level)
{
    int i,ret=0;

    level += (NUM_CONTRAST_LEVELS / 2 + 1);
    if (level < 0 || level > NUM_CONTRAST_LEVELS) {
        return -1;
    }

    /* Switch to DSP register bank */
    ret |= SCCB_Write(BANK_SEL, BANK_SEL_DSP);

    /* Write contrast registers */
    for (i=0; i<sizeof(contrast_regs[0])/sizeof(contrast_regs[0][0]); i++) {
        ret |= SCCB_Write(contrast_regs[0][i], contrast_regs[level][i]);
    }

    return ret;
}

static int set_brightness(int level)
{
    int i,ret=0;

    level += (NUM_BRIGHTNESS_LEVELS / 2 + 1);
    if (level < 0 || level > NUM_BRIGHTNESS_LEVELS) {
        return -1;
    }

    /* Switch to DSP register bank */
    ret |= SCCB_Write(BANK_SEL, BANK_SEL_DSP);

    /* Write brightness registers */
    for (i=0; i<sizeof(brightness_regs[0])/sizeof(brightness_regs[0][0]); i++) {
        ret |= SCCB_Write(brightness_regs[0][i], brightness_regs[level][i]);
    }

    return ret;
}

static int set_saturation(int level)
{
    int i,ret=0;

    level += (NUM_SATURATION_LEVELS / 2 + 1);
    if (level < 0 || level > NUM_SATURATION_LEVELS) {
        return -1;
    }

    /* Switch to DSP register bank */
    ret |= SCCB_Write(BANK_SEL, BANK_SEL_DSP);

    /* Write contrast registers */
    for (i=0; i<sizeof(saturation_regs[0])/sizeof(saturation_regs[0][0]); i++) {
        ret |= SCCB_Write(saturation_regs[0][i], saturation_regs[level][i]);
    }

    return ret;
}
static int set_effects(int level)
{
    int i,ret=0;

    
    if (level < 0 || level > NUM_SPECIAL_EFFECTS) {
        return -1;
    }

    /* Switch to DSP register bank */
    ret |= SCCB_Write(BANK_SEL, BANK_SEL_DSP);

    /* Write contrast registers */
    for (i=0; i<sizeof(effects_regs[0])/sizeof(effects_regs[0][0]); i++) {
        ret |= SCCB_Write(effects_regs[0][i], effects_regs[level][i]);
    }

    return ret;
}


static int set_gainceiling(enum sensor_gainceiling gainceiling)
{
    int ret =0;

    /* Switch to SENSOR register bank */
    ret |= SCCB_Write(BANK_SEL, BANK_SEL_SENSOR);

    /* Write gain ceiling register */
    ret |= SCCB_Write(COM9, COM9_AGC_SET(gainceiling));

    return ret;
}

static int set_shutterspeed(unsigned short speed)
{
	int ret=0;
	unsigned char v;
	unsigned short temp16;
		//Shutter = (reg0x45 & 0x3f) << 10 + reg0x10<<2 + (reg0x04 & 0x03);
	
  /* Switch to DSP register bank */
  ret |= SCCB_Write(BANK_SEL, BANK_SEL_SENSOR);

	
	//Set register 04 bits 0,1
	DCMI_SingleRandomRead(0x04, &v);
	v &= 0xFC;
	v |= (unsigned char)speed & 0x3;
	DCMI_SingleRandomWrite(0x04, v);

	//Set register 10(AEC) bits 2-9
	temp16 = speed>>2;
	DCMI_SingleRandomWrite(AEC, (unsigned char)(temp16));
	
	
	//Set register(45) bit 15:10
	DCMI_SingleRandomRead(REG45, &v);
	v &= 0x3f;
	temp16 = speed>>10;
	ret |= SCCB_Write(REG45,(unsigned char)temp16);

	return ret;
}
static int get_shutterspeed(void)
{
	unsigned shutter;
	
	unsigned char r[3];
		  /* Switch to DSP register bank */
  SCCB_Write(BANK_SEL, BANK_SEL_SENSOR);
		//Shutter = (reg0x45 & 0x3f) << 10 + reg0x10<<2 + (reg0x04 & 0x03);
	DCMI_SingleRandomRead(0x04,  &r[0]);
	DCMI_SingleRandomRead(AEC,   &r[1]);
	DCMI_SingleRandomRead(REG45, &r[2]);
	
	shutter = ((unsigned short)r[2] & 0x3f) << 10 | (unsigned short)r[1]<<2 | ((unsigned short)r[0] & 0x03);
	
	return shutter;
	
}
static int set_gain(unsigned char gain)//1-32
{

	return SCCB_Write(GAIN,gain); 
	
}

static int get_exposureval(void)
{
	unsigned shutter;
	
	unsigned char r[3];
		  /* Switch to DSP register bank */
  SCCB_Write(BANK_SEL, BANK_SEL_SENSOR);
		//Shutter = (reg0x45 & 0x3f) << 10 + reg0x10<<2 + (reg0x04 & 0x03);
	DCMI_SingleRandomRead(AEW,  &r[0]);
	DCMI_SingleRandomRead(AEB,   &r[1]);
	DCMI_SingleRandomRead(REG45, &r[2]);
	
	shutter = ((unsigned short)r[2] & 0x3f) << 10 | (unsigned short)r[1]<<2 | ((unsigned short)r[0] & 0x03);
	
	return shutter;
	
}
static int set_lightmode(unsigned char mode)
{
	int ret=0;
	/* Switch to DSP register bank */
	ret |= SCCB_Write(BANK_SEL, BANK_SEL_DSP);

	
	
	if( mode == LIGHT_MODE_AUTO )
	{
		SCCB_Write(0xc7, 0x00); //AWB on
	}
	else if( mode == LIGHT_MODE_SUNNY)
	{
		SCCB_Write(0xc7, 0x40); //AWB off
		SCCB_Write(0xcc, 0x5e);
		SCCB_Write(0xcd, 0x41);
		SCCB_Write(0xce, 0x54);
	}
	else if( mode == LIGHT_MODE_CLOUDY)
	{
		SCCB_Write(0xc7, 0x40); //AWB off
		SCCB_Write(0xcc, 0x65);
		SCCB_Write(0xcd, 0x41);
		SCCB_Write(0xce, 0x4f);
	}
	else if(mode == LIGHT_MODE_OFFICE)
	{
		SCCB_Write(0xc7, 0x40); //AWB off
		SCCB_Write(0xcc, 0x52);
		SCCB_Write(0xcd, 0x41);
		SCCB_Write(0xce, 0x66);
	}
	else if(mode == LIGHT_MODE_HOME)
	{
		SCCB_Write(0xc7, 0x40); //AWB off
		SCCB_Write(0xcc, 0x42);
		SCCB_Write(0xcd, 0x3f);
		SCCB_Write(0xce, 0x71);
	}
		
return ret;
}

static int set_quality(int qs)
{
    int ret=0;

    /* Switch to DSP register bank */
    ret |= SCCB_Write(BANK_SEL, BANK_SEL_DSP);

    /* Write QS register */
    ret |= SCCB_Write(QS, qs);

    return ret;
}

void OV2640_JPEGFullInit(void)
{
	unsigned i;
	const uint8_t (*regs)[2];
	
	
	 SCCB_Write(BANK_SEL, BANK_SEL_SENSOR);
   SCCB_Write(COM7, COM7_SRST);
   /* delay n ms */
   Delayms(10); 
		//Write initregs;
		i = 0;
		regs = default_regs;
		/* Write initial regsiters */
		while (regs[i][0]) {
				SCCB_Write(regs[i][0], regs[i][1]);
				i++;
		}

		i = 0;
		regs = svga_regs;
		//regs = uxga_regs;
		/* Write DSP input regsiters */
		while (regs[i][0]) {
				SCCB_Write(regs[i][0], regs[i][1]);
				i++;
		}
	//SET PIXEL FORMAT
		i = 0;
		regs = jpeg_regs;
		
		while (regs[i][0]) {
        SCCB_Write(regs[i][0], regs[i][1]);
        i++;
    }
		
		
	set_pixformat(PIXFORMAT_JPEG);	
	
		
		
				i = 0;
		regs = test;
		
		while (regs[i][0]) {
        SCCB_Write(regs[i][0], regs[i][1]);
        i++;
    }
		
	///set_framesize(FRAMESIZE_UXGA);
		
	//set_gainceiling(GAINCEILING_8X);
		
	set_contrast(3);
		
	set_brightness(1);
		
	//set_quality(0x8);		
	set_quality(0x20);	
	//set_quality(0x1f);	
		
	//set_saturation(1);
		
	//set_lightmode(LIGHT_MODE_OFFICE);	
		
	//set_effects(SPECIAL_EFFECT_BW);	
		
	set_exposure(AUTOEXPOSURE_LEVEL3);// does not effect in man exposure	
  //set_gain(GAIN_32X);//1-32
	//set_shutterspeed(200);	//default 312
	//set_shutterspeed(600);	//default 312	
		
	//shutsp = get_shutterspeed();	

	//set_saturation(1);
	
	SCCB_Write(BANK_SEL, BANK_SEL_DSP);
	SCCB_Write(R_DVP_SP, 0);//output speed control
  SCCB_Write(BANK_SEL, BANK_SEL_SENSOR);//01
	SCCB_Write(COM2, COM2_OUT_DRIVE_4x);
	SCCB_Write(CLKRC, 0X80);
}
