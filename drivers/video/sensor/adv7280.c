

#include <common.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>  // include all mx6dq pins def
#include <asm/errno.h>
#include <asm/gpio.h>
#include <linux/types.h>
#include <video/adv7280.h>
#include <i2c.h>
#include <linux/compiler.h>
#include "../ipu_sdk/ipu_common.h"
#include "../ipu_sdk/ips_disp_panel.h"
#include "adv7280_def.h"
#include <asm/imx-common/video.h>
#include "board_camera.h"

//! @brief The common structure for accessing adv7180 I2C interface.
typedef struct {
    uint8_t addr;	//!< adv7180 register address
    uint8_t value;	//!< value along with the register address
} t_adv7180_i2c_reg_param;


//wyb
/* IPU CSI0 IOMUX pads */
iomux_v3_cfg_t const csi0_pads[] = {
	MX6_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,	    /* CSI0_MCLK/HSYNC */
	MX6_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,	/* CSI0_PIXCLK */
	MX6_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,    /* CSI0_VSYNC */
    MX6_PAD_CSI0_DAT12__IPU1_CSI0_DATA12,	/* CSI0_DAT12 */
	MX6_PAD_CSI0_DAT13__IPU1_CSI0_DATA13,	/* CSI0_DAT13 */
	MX6_PAD_CSI0_DAT14__IPU1_CSI0_DATA14,	/* CSI0_DAT14 */
	MX6_PAD_CSI0_DAT15__IPU1_CSI0_DATA15,	/* CSI0_DAT15 */
	MX6_PAD_CSI0_DAT16__IPU1_CSI0_DATA16,	/* CSI0_DAT16 */
	MX6_PAD_CSI0_DAT17__IPU1_CSI0_DATA17,	/* CSI0_DAT17 */
	MX6_PAD_CSI0_DAT18__IPU1_CSI0_DATA18,	/* CSI0_DAT18 */
	MX6_PAD_CSI0_DAT19__IPU1_CSI0_DATA19,	/* CSI0_DAT19 */
};

/* IPU CSI1 IOMUX pads */
iomux_v3_cfg_t const csi1_pads[] = {
	MX6_PAD_EIM_DA11__IPU2_CSI1_HSYNC,	/* CSI1_MCLK/HSYNC */
	MX6_PAD_EIM_A16__IPU2_CSI1_PIXCLK,	/* CSI1_PIXCLK */
	MX6_PAD_EIM_DA12__IPU2_CSI1_VSYNC,  /* CSI1_VSYNC */
    MX6_PAD_EIM_A17__IPU2_CSI1_DATA12,	/* CSI1_DAT12 */
	MX6_PAD_EIM_A18__IPU2_CSI1_DATA13,	/* CSI1_DAT13 */
	MX6_PAD_EIM_A19__IPU2_CSI1_DATA14,	/* CSI1_DAT14 */
	MX6_PAD_EIM_A20__IPU2_CSI1_DATA15,	/* CSI1_DAT15 */
	MX6_PAD_EIM_A21__IPU2_CSI1_DATA16,	/* CSI1_DAT16 */
	MX6_PAD_EIM_A22__IPU2_CSI1_DATA17,	/* CSI1_DAT17 */
	MX6_PAD_EIM_A23__IPU2_CSI1_DATA18,	/* CSI1_DAT18 */
	MX6_PAD_EIM_A24__IPU2_CSI1_DATA19,	/* CSI1_DAT19 */
};


// #define BT656

/*! Description of video formats supported.
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
video_fmt_t video_fmts[] = {
	{			/*! NTSC */
	 .fmt_id = ADV7180_NTSC,
	 .name = "NTSC",
	 .raw_width = 720,	/* SENS_FRM_WIDTH */
	 .raw_height = 525,	/* SENS_FRM_HEIGHT */
	 .active_width = 720,	/* ACT_FRM_WIDTH plus 1 */
	 .active_height = 480,	/* ACT_FRM_WIDTH plus 1 */
	 },
	{			/*! (B, G, H, I, N) PAL */
	 .fmt_id = ADV7180_PAL,
	 .name = "PAL",
	 .raw_width = 804,  // 864, 1728 clks??
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 },
	{			/*! Unlocked standard */
	 .fmt_id = ADV7180_NOT_LOCKED,
	 .name = "Autodetect",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 },
};


#define HX7097_CHIP_ADDR 0x48  //i2c addr 0x90

int HX7097_reg_write(unsigned char i2cbus, u32 reg, u32 val)
{
	unsigned char buf[4] = { 0 };

	I2C_SET_BUS(i2cbus);

	buf[0] = cpu_to_le32(val) & 0xff;
	
	if (i2c_write(HX7097_CHIP_ADDR, reg, 1, buf, 1))
		return -1;

	return 0;

}

void HX7097_init(unsigned char i2cbus)
{

	HX7097_reg_write(i2cbus, 0x00, 0x04); // addr, reg
	HX7097_reg_write(i2cbus, 0x02, 0x04); // addr, reg
	HX7097_reg_write(i2cbus, 0x03, 0x01); // addr, reg
	HX7097_reg_write(i2cbus, 0x04, 0x02); // addr, reg

	HX7097_reg_write(i2cbus, 0x09, 0xff); // addr, reg
	HX7097_reg_write(i2cbus, 0x0a, 0xa4); // addr, reg
	HX7097_reg_write(i2cbus, 0x0b, 0x6c); // addr, reg
	HX7097_reg_write(i2cbus, 0x0c, 0x3f); // addr, reg
	HX7097_reg_write(i2cbus, 0x0d, 0x10); // addr, reg
	HX7097_reg_write(i2cbus, 0x0e, 0x0a); // addr, reg
	HX7097_reg_write(i2cbus, 0x0f, 0x00); // addr, reg
	HX7097_reg_write(i2cbus, 0x10, 0x5b); // addr, reg
	HX7097_reg_write(i2cbus, 0x11, 0x93); // addr, reg
	HX7097_reg_write(i2cbus, 0x12, 0xc0); // addr, reg
	HX7097_reg_write(i2cbus, 0x13, 0xef); // addr, reg
	HX7097_reg_write(i2cbus, 0x14, 0xff); // addr, reg

	HX7097_reg_write(i2cbus, 0x15, 0xff); // addr, reg
	HX7097_reg_write(i2cbus, 0x16, 0xa4); // addr, reg
	HX7097_reg_write(i2cbus, 0x17, 0x6c); // addr, reg
	HX7097_reg_write(i2cbus, 0x18, 0x3f); // addr, reg
	HX7097_reg_write(i2cbus, 0x19, 0x10); // addr, reg
	HX7097_reg_write(i2cbus, 0x1a, 0x0a); // addr, reg
	HX7097_reg_write(i2cbus, 0x1b, 0x00); // addr, reg
	HX7097_reg_write(i2cbus, 0x1c, 0x5b); // addr, reg
	HX7097_reg_write(i2cbus, 0x1d, 0x93); // addr, reg
	HX7097_reg_write(i2cbus, 0x1e, 0xc0); // addr, reg
	HX7097_reg_write(i2cbus, 0x1f, 0xef); // addr, reg
	HX7097_reg_write(i2cbus, 0x20, 0xff); // addr, reg

	HX7097_reg_write(i2cbus, 0x21, 0xff); // addr, reg
	HX7097_reg_write(i2cbus, 0x22, 0xa4); // addr, reg
	HX7097_reg_write(i2cbus, 0x23, 0x6c); // addr, reg
	HX7097_reg_write(i2cbus, 0x24, 0x3f); // addr, reg
	HX7097_reg_write(i2cbus, 0x25, 0x10); // addr, reg
	HX7097_reg_write(i2cbus, 0x26, 0x0a); // addr, reg
	HX7097_reg_write(i2cbus, 0x27, 0x00); // addr, reg
	HX7097_reg_write(i2cbus, 0x28, 0x5b); // addr, reg
	HX7097_reg_write(i2cbus, 0x29, 0x93); // addr, reg
	HX7097_reg_write(i2cbus, 0x2a, 0xc0); // addr, reg
	HX7097_reg_write(i2cbus, 0x2b, 0xef); // addr, reg
	HX7097_reg_write(i2cbus, 0x2c, 0xff); // addr, reg
	
}


#define ADV7180_CHIP_ADDR 0x20

int adv7280_reg_read(unsigned char i2cbus, u32 reg, u32 *val)
{
	unsigned char buf[4] = { 0 };
	u32 ret_val = 0;

	I2C_SET_BUS(i2cbus);  // set current bus i2c 

	if (i2c_read(ADV7180_CHIP_ADDR, reg, 1, buf, 1))
		return -1;

	ret_val = le32_to_cpu(buf[0]);

	memcpy(val, &ret_val, sizeof(ret_val));

	return 0;
}

int adv7280_reg_write(unsigned char i2cbus, u32 reg, u32 val)
{
	unsigned char buf[4] = { 0 };

	I2C_SET_BUS(i2cbus);

	buf[0] = cpu_to_le32(val) & 0xff;
	
	if (i2c_write(ADV7180_CHIP_ADDR, reg, 1, buf, 1))
		return -1;

	return 0;

}

int adv7280_write(unsigned char i2cbus, uint8_t chip, u32 reg, u32 val)
{
	unsigned char buf[4] = { 0 };

	I2C_SET_BUS(i2cbus);

	buf[0] = cpu_to_le32(val) & 0xff;
	
	if (i2c_write(chip, reg, 1, buf, 1))
		return -1;

	return 0;

}



// wyb
// ##CVBS AUTODETECT##
//:Autodetect CVBS Single Ended In Ain 1, YPrPb Out:
//delay 10 ;
//42 0F 00 ; Exit Power Down Mode [ADV7280 writes begin]
//42 00 00 ; CVBS in on AIN1
//42 0E 80 ; ADI Required Write 
//42 9C 00 ; ADI Required Write 
//42 9C FF ; ADI Required Write 
//42 0E 00 ; Enter User Sub Map
//42 03 0C ; Enable Pixel & Sync output drivers
//42 04 07 ; Power-up INTRQ, HS & VS pads
//42 13 00 ; Enable INTRQ output driver
//42 17 41 ; Enable SH1
//42 1D 40 ; Enable LLC output driver
//42 52 CD ; ADI Required Write 
//42 80 51 ; ADI Required Write
//42 81 51 ; ADI Required Write
//42 82 68 ; ADI Required Write [ADV7280 writes finished]

int adv7280_init(unsigned char i2cbus)
{
	int ret;
	int i = 0;
	// int temp;
	unsigned int reg;
	t_adv7180_i2c_reg_param *setting;

	// read chip id
	adv7280_reg_read(i2cbus, 0x11, &reg);
	
	printf("i2c bus %x , adv7280: ID=0x%02x\n", i2cbus,reg);

	/* init adv7280 */
	// chip power up
	adv7280_reg_write(i2cbus, 0x0f, 0x00); // addr, reg
	// input control cvbs Ain1
	adv7280_reg_write(i2cbus, 0x00, 0x00); // addr, reg
	// output control, enable pixel and sync output
	adv7280_reg_write(i2cbus, 0x03, 0x0c); // addr, reg
	// enable LLC pixel clk
	adv7280_reg_write(i2cbus, 0x1d, 0x40); // addr, reg
	// enable auto dectection
	// adv7280_reg_write(i2cbus, 0x07, 0x7f);
#ifdef BT656

    // enter VDP sub map
	adv7280_reg_write(i2cbus, 0x0e, 0x80); 
	adv7280_reg_write(i2cbus, 0x9c, 0x00); 
	adv7280_reg_write(i2cbus, 0x9c, 0xff);
	// enter user map
	adv7280_reg_write(i2cbus, 0x0e, 0x00);

    // bt656, SFL disable, blank when VBI, HS,VS tristated
	adv7280_reg_write(i2cbus, 0x04, 0x07);
	// enable SH1
	adv7280_reg_write(i2cbus, 0x17, 0x41);
    // Enable INTRQ output driver
	adv7280_reg_write(i2cbus, 0x13, 0x00);

	// VS or FIELD
	// adv7280_reg_write(i2cbus,0x6b,0x11);  // VS output

    // ADI required write
	adv7280_reg_write(i2cbus, 0x52, 0xcd);
	adv7280_reg_write(i2cbus, 0x80, 0x51);
	adv7280_reg_write(i2cbus, 0x81, 0x51);
	adv7280_reg_write(i2cbus, 0x82, 0x68);

	// set vpp map
	adv7280_reg_write(i2cbus, 0xfd, 0x84);
	// 0x84 >> 1, VDP sub
	adv7280_write(i2cbus, 0x42, 0xa3, 0x00); //adi required
	// enable advanced timing mode
	adv7280_write(i2cbus, 0x42, 0x5b, 0x00);
	// enable I2P converter
	adv7280_write(i2cbus, 0x42, 0x55, 0x80);

#else 
	
	// adv7280_reg_write(i2cbus, 0x0c, 0x37);    // blue screen output
	// adv7280_reg_write(i2cbus, 0x0d, 0xc7);    // default UV c0,70
	// adv7280_reg_write(i2cbus, 0x04, 0xcd); // force vs hs output

    // PAL VS HS timing, timing vs hs or BT656
	adv7280_reg_write(i2cbus,0x31,0x1a);
	adv7280_reg_write(i2cbus,0x32,0x81);
	adv7280_reg_write(i2cbus,0x33,0x84);
	adv7280_reg_write(i2cbus,0x34,0x00);
	adv7280_reg_write(i2cbus,0x35,0x00);
	adv7280_reg_write(i2cbus,0x36,0x7d);

	// hs,vs,clk, polarity ,7,5,0bit, PAL = 0xa1
	// HS active high, VS active low, field active high, clk pos
	adv7280_reg_write(i2cbus,0x37,0xa1);
	// VS or FIELD
	// adv7280_reg_write(i2cbus,0x6b,0x12);  // FIELD
	adv7280_reg_write(i2cbus,0x6b,0x11);  // VS
	// PAL V/F bit
	adv7280_reg_write(i2cbus,0xe8,0x41);
	adv7280_reg_write(i2cbus,0xe9,0x84);
	adv7280_reg_write(i2cbus,0xea,0x06);  // 

		// set vpp map
	adv7280_reg_write(i2cbus, 0xfd, 0x84);
	// 0x84 >> 1, VDP sub
	adv7280_write(i2cbus, 0x42, 0xa3, 0x00); //adi required
	// enable advanced timing mode
	adv7280_write(i2cbus, 0x42, 0x5b, 0x00);
	// enable I2P converter
	adv7280_write(i2cbus, 0x42, 0x55, 0x80);

#endif
	// read input format
	adv7280_reg_read(i2cbus, 0x10, &reg);
	if(reg&0x01)
	{
		ret = 1; // have input signal
		// AD_RESULT[2:0] = 100 --> PAL B/G/H/I/D
		printf("i2c bus %x , adv7280 has input reg=%x\n", i2cbus, reg);
		return ret;
	}
	else
	{
		ret = 0;
		printf("i2c bus %x , adv7280 did not has input reg=%x\n", i2cbus,reg);
		return ret;		// do not have input signal
		
	}

}


int32_t adv7180_get_std(unsigned char i2cbus)
{
	int32_t ret;
	// int temp;
	unsigned int reg;

	// read input format
	adv7280_reg_read(i2cbus, 0x10, &reg);
	if(reg&0x40)
	{
		ret = ADV7180_PAL;
		// AD_RESULT[2:0] = 100 --> PAL B/G/H/I/D
		printf("adv7280  input is PAL\n");
		return ret;
	}
	else
	{
	// x 0 0 0 x x x x is NTSC
		ret = ADV7180_NTSC;
		printf("adv7280  input is NTSC\n");
		return ret;		//
		
	}

}


void load_panel_settings(ips_dev_panel_t* ips_panel)
{
	char const *panel;

	panel = displays[4].mode.name;

    // ips_panel->panel_name = panel;
	ips_panel->width = displays[4].mode.xres;
	ips_panel->height = displays[4].mode.yres;
	ips_panel->colorimetry = DCMAP_RGB888;
	
/*
		.bus	= -1,
		.addr	= 0,
		.pixfmt = IPU_PIX_FMT_RGB24,
		.detect = detect_hdmi,
		.enable = do_enable_hdmi,
		.mode	= {
	// wyb

		 .name           = "HDMI",
		.refresh        = 60,
	    .xres			= 1920, //1080P
	    .yres			= 1080,
	    .pixclock		= 6734, // 6734 ps, 148500kHz
	    .left_margin	= 148,
	    .right_margin	= 88, 
	    .upper_margin	= 36, 
	    .lower_margin	= 4, 
	    .hsync_len		= 44, 
	    .vsync_len		= 5,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED

"HDMI 1080P 60Hz",		   // name
HDMI_1080P60,			   // panel id flag
DISP_DEV_HDMI,					 // panel type
DCMAP_RGB888,			   // data format for panel
60, 					   // refresh rate
1920,					   // panel width
1080,					   //panel height
148500000,				   // pixel clock frequency
192,					   // hsync start width
44, 					   // hsync width
88, 					   // hsyn back width
41, 					   // vysnc start width
5,						   // vsync width
4,						   // vsync back width
0,						   // delay from hsync to vsync
0,						   // interlaced mode
1,						   // clock selection, external
0,						   // clock polarity
1,						   // hsync polarity
1,						   // vync polarity
1,						   // drdy polarity
0,						   // data polarity

  x     y  hz  pixclk ps/kHz   le   ri  up  lo   hs vs  s  vmode 

{1920, 1080, 60,  6734, 148500, 148,  88, 36,  4,  44, 5, FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT, FB_VMODE_NONINTERLACED},

			
*/	






}



int32_t adv7280_capture(unsigned char i2cbus)
{
    int32_t ipu_index = i2cbus + 1;  // ipu_index 1,2
    int adv7180_std, csi_interface;
	int gated_mode;
    ips_dev_panel_t panel;
    video_fmt_t *in_fmt;
    // uint8_t revchar;
    int32_t ret = 0;

    // io config 
    if(i2cbus == 0)
    {
    	/* Setup CSI0 camera */
		// imx_iomux_v3_setup_multiple_pads(csi0_pads, ARRAY_SIZE(csi0_pads));
		csi_port0_iomux_config();
    }
	else if(i2cbus == 1)
	{		
		/* Setup CSI1 camera */
		// imx_iomux_v3_setup_multiple_pads(csi1_pads, ARRAY_SIZE(csi1_pads));
		csi_port1_iomux_config();
	}

    /*step 1: get panel */
   // panel = search_panel("HDMI 1080P 60Hz");
   // panel->panel_init(&ipu_index);
    load_panel_settings(&panel);

	HX7097_init(2);  //  i2c3

    /*step 2: setup adv7180 */
	ret = adv7280_init(i2cbus);

	adv7180_std = adv7180_get_std(i2cbus);
    // gated_mode = adv7280_is_interlaced_mode();
    gated_mode = 1;

//	if ((adv7180_std == ADV7180_NTSC) && (gated_mode == 1)) {
//       csi_interface = CSI_BT656_NTSC_INTERLACED;  /*interlaced BT656 */
//    } else if ((adv7180_std == ADV7180_PAL) && (gated_mode == 1)) {
//        csi_interface = CSI_BT656_PAL_INTERLACED;   /*interlaced BT656 */		
//    } else {
//        printf("Not supported BT656 input!\n");      
//    }

	 csi_interface = CSI_PARALLEL;
	// csi_interface = CSI_BT656_PAL_INTERLACED;
	//   csi_interface =  CSI_BT656_PAL_PROGRESSIVE;
	// csi_interface =  CSI_TEST_MODE;

	// csi_interface = CSI_BT656_PAL_INTERLACED;
    // in_fmt = &video_fmts[adv7180_std];
	in_fmt = &video_fmts[1];  // PAL timing

	// printf("Do you want to enable deinterlace processing? y or n\n");
	   csi_vdi_direct_path = 0;  // no deinterlace
	// csi_vdi_direct_path = 1;  // interlace

	/*step 3: setup IPU: from csi to display */
   // ipu_sw_reset(1, 1000);
   // ipu_reset();

	udelay(20000);

	// ipu_reg_test(ipu_index);

    ipu_capture_setup(ipu_index, csi_interface, in_fmt->raw_width, in_fmt->raw_height,
                      in_fmt->active_width, in_fmt->active_height, &panel);
	// enable dp dc di dmfc
    // ipu_enable_display(1);

    // printf("Do you see the captured image (y or n)?\n");

	return ret;

}














