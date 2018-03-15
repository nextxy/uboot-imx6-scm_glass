

#include <common.h>
#include <linux/types.h>
#include <linux/err.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <div64.h>
#include <video_fb.h>

#include <imx6dq/regssrc.h>
#include <imx6dq/interrupt.h>
#include <imx6dq/gic.h>

#include "../ipu/ipu.h"
#include "ipu_common.h"
#include "ipu_idmac.h"
#include "ipu_csi.h"
#include "ipu_vdi.h"
#include "ipu_dmfc.h"
#include "ipu_dp.h"
#include "ipu_di.h"
#include "ipu_dc.h"
#include "ipu_ic.h"
#include "../cfb_console.h"
#include "../gui/gui.h"


int csi_vdi_direct_path = 0;
int csi_dma_band_mode = 0;
extern GraphicDevice *pGD;	/* Pointer to Graphic array */


void ipu_ch0_eobnd_interrupt_register(int ipu_index);


//globals used for gic_test
unsigned int gic_ipu1_eof_flag;

void gic_ipu1_handler(void)
{
    // printf("In ipu_test_handler()\n");

    // read idmac_EOF_0
    if(ipu_read(1, IPU_IPU_INT_STAT_1__ADDR) & 0x1) {
		gic_ipu1_eof_flag = 1;			  // eof flag
		// clear EOF intterupt
    	ipu_write_field(1, IPU_IPU_INT_STAT_1__IDMAC_EOF_0, 1);
    }
	
	// refresh_screen(1);
		
}

// intterrupt init
void gic_ipu_init(void)
{

    // init enable interrupt
    gic_init_sdk();

    // register and enable sgi isr
    register_interrupt_routine(IMX_INT_IPU1_FUNC, gic_ipu1_handler);
    enable_interrupt(IMX_INT_IPU1_FUNC, CPU_0, 0);

    gic_ipu1_eof_flag = 0;

   // printf("Sending SGI\n");
   // gic_send_sgi(SW_INTERRUPT_3, 1, kGicSgiFilter_UseTargetList);
}

void gic_ipu_test(void)
{
	uint32_t  timeout = 300000;  //3s

	// printf("Waiting ipu interrupt test\n");
	while (!(gic_ipu1_eof_flag)) {
		udelay(10);
		if (timeout <= 0) {
			printf("ipu wait interrupt time out.\n");
			break;
		}			 
		timeout--;	
	}

	printf("ipu int test end\n");

}


inline int32_t need_csc(int32_t i, int32_t o)
{
    if ((((i & 0xF) == (INTERLEAVED_RGB)) && (o > DCMAP_BRG888))
        || (((i & 0xF) != INTERLEAVED_RGB) && (o <= DCMAP_BRG888)))
        return 1;
    else
        return 0;
}

uint32_t REGS_IPU_BASEaaa(uint32_t index_ipu)
{
	if(index_ipu == 1)
		return REGS_IPU1_BASE; // 0x02600000;
	else if(index_ipu == 2)
		return REGS_IPU2_BASE; // 0x02a00000;

}



/*!
 * write field of ipu registers, without affecting other bits.
 *
 * @param	ipu_index:	ipu index
 * @param	ID_addr:	register address
 * @param	ID_mask:	fields position
 * @param	data:		the value of input
 */
void ipu_write_field(int32_t ipu_index, uint32_t ID_addr, uint32_t ID_mask, uint32_t data)
{
	uint32_t rdata;

	// ipu_index 1 or 2
	ID_addr += REGS_IPU_BASE(ipu_index);

	// rdata = readl(ID_addr);
	rdata = __raw_readl(ID_addr);
	rdata &= ~ID_mask;
	rdata |= (data * (ID_mask & -ID_mask)) & ID_mask;
	__raw_writel(rdata, ID_addr);
	// writel(rdata, ID_addr);
}

/*!
 * write field of ipu registers, without affecting other bits.
 *
 * @param	ipu_index:	ipu index
 * @param	ID_addr:	register address
 */
uint32_t ipu_read(int32_t ipu_index, uint32_t ID_addr)
{
	uint32_t rdata;

	ID_addr += REGS_IPU_BASE(ipu_index);

	rdata = __raw_readl(ID_addr);	
	// rdata = readl(ID_addr);

	return rdata;
}

/*!
 * write field of ipu registers, without affecting other bits.
 *
 * @param	ipu_index:	ipu index
 * @param	ID_addr:	register address
 * @param	ID_mask:	fields position
 * @param	data:		the value of input
 */
void ipu_write(int32_t ipu_index, uint32_t ID_addr, uint32_t data)
{
	ID_addr += REGS_IPU_BASE(ipu_index);

	__raw_writel(data, ID_addr);
	// writel(data, ID_addr);
}


/*!
 * enable submodules of IPU to establish the data path.
 *
 * @param	ipu_index:	ipu index
 */
void ipu_enable_display(int32_t ipu_index)
{
    /*enable all the related submodules. */
    ipu_write_field(ipu_index, IPU_IPU_CONF__DI0_EN, 1);
    ipu_write_field(ipu_index, IPU_IPU_CONF__DP_EN, 1);
    ipu_write_field(ipu_index, IPU_IPU_CONF__DC_EN, 1);
    ipu_write_field(ipu_index, IPU_IPU_CONF__DMFC_EN, 1);
}

/*!
 * disable submodules of IPU to establish the data path.
 *
 * @param	ipu_index:	ipu index
 */
void ipu_disable_display(int32_t ipu_index)
{
    /*enable all the related submodules. */
    ipu_write_field(ipu_index, IPU_IPU_CONF__DI0_EN, 0);
    ipu_write_field(ipu_index, IPU_IPU_CONF__DI1_EN, 0);
    ipu_write_field(ipu_index, IPU_IPU_CONF__DP_EN, 0);
    ipu_write_field(ipu_index, IPU_IPU_CONF__DC_EN, 0);
    ipu_write_field(ipu_index, IPU_IPU_CONF__DMFC_EN, 0);
}

/*!
 * reset ipu by SRC(system reset controller)
 *
 * @param	ipu_index:	ipu index
 * @param	timeout:    time out setting for ipu reset
 *
 * @return	true for success, others for time out.
 */
int32_t ipu_sw_reset(int32_t ipu_index, int32_t timeout)
{
    uint32_t tmpVal;
    int32_t ipuOffset = 0x3;

    if (ipu_index == 1)
        ipuOffset = 0x3;
    else
        ipuOffset = 0xC;
    tmpVal = readl(SRC_BASE_ADDR);
    writel(tmpVal | (0x1 << ipuOffset), SRC_BASE_ADDR);

    while (timeout > 0) {
        tmpVal = readl(SRC_BASE_ADDR) & (0x1 << ipuOffset);
        timeout--;

        if (tmpVal == 0)
            return true;
    }

    printf("Error: ipu software reset time out!!\n");
    return false;
}


// YUV422 --> RG565
// U,Y,V,Y1  0xc034_7034  blue 
// RGB565    0x1954_1954  blue
void YUV422_to_RGB565(u32* ycbcr,u32* rgb_buf,u32 len)
{
	u32 i;
	u32 temp_data;
	int y,u,v;
	int y1,u1,v1;
	int r,g,b;
	int r1,g1,b1;
	int temp1,temp2,temp3;
	u16 rgb = 0;
	u16 rgb1 = 0;
	u16 temp_r = 0,temp_g = 0,temp_b = 0;
	for(i = 0;i < len/4;i++)
	{
		
		// μ??·′óD?μ?′óμ???áD?3Dò?a Yn, Vn, Yn+1, Un
	  temp_data = *(ycbcr+i);
    y = (u8)(temp_data>>16); 
    // u = (u8)(temp_data>>24);
	//	v = (u8)(temp_data>>8);
	 v = (u8)(temp_data>>24);
		u = (u8)(temp_data>>8);

		y1 = (u8)(temp_data);		
		u = u - 128;
		v = v - 128;
		u1 = u;
		v1 = v;
		
		// yuv422 to RGB ×a??
		temp1 = (v+ ((v*103)>>8));
		temp2 = (((u*88)>>8) + ((v*183)>>8));
		temp3 = (u+((u*198)>>8));
		r = (y + temp1);
		g = (y - temp2);
		b = (y + temp3);		
		if(r < 0)r = 0;
		if(r > 255) r = 255;
		if(g < 0)g = 0;
		if(g > 255) g = 255;
		if(b < 0)b = 0;
		if(b > 255) b = 255;
		
		r1 = (y1 + temp1);
		g1 = (y1 - temp2);
		b1 = (y1 + temp3);		
		if(r1 < 0)r1 = 0;
		if(r1 > 255) r1 = 255;
		if(g1 < 0)g1 = 0;
		if(g1 > 255) g1 = 255;
		if(b1 < 0)b1 = 0;
		if(b1 > 255) b1 = 255;
		
    //  RGB888 ×a RGB565		
		temp_r = (r&0xf8)<<8;
		temp_g = (g&0xfc)<<3;
		temp_b = (b&0xf8)>>3;
		rgb = temp_r + temp_g + temp_b;
		
		temp_r = 0,temp_g = 0,temp_b = 0;
		temp_r = (r1&0xf8)<<8;
		temp_g = (g1&0xfc)<<3;
		temp_b = (b1&0xf8)>>3;
		rgb1 = temp_r + temp_g + temp_b;
		temp_data = (((u32)(rgb)<<16) | rgb1);
		*(rgb_buf+i) = temp_data;	
	}	
}	




/***********************************************************/

#if 1
/*!
 * display function HW configuration for IPU.
 *
 * @param   ipu_index ipu index
 * @param   panel ipu panel configuration data structure
 * @param   mem_colorimetry colorimetry configuration
 * @param   csc_type color space conversion type
 */
void ipu_display_setup(uint32_t ipu_index, uint32_t mem_addr0, uint32_t mem_addr1,
                       uint32_t mem_colorimetry, ips_dev_panel_t * panel)
{
    uint32_t channel = MEM_TO_DP_BG_CH23;
    uint32_t di = 0;
    uint32_t in_type, out_type, csc_type = NO_CSC;

    /*step1: determine CSC type according input colorimetry and output colorimetry */
    in_type = ((mem_colorimetry & 0xF) == INTERLEAVED_RGB) ? RGB : YUV;
    switch (panel->colorimetry) {
    case DCMAP_YUV888:
    case DCMAP_UVY888:
    case DCMAP_VYU888:
    case DCMAP_YUVA8888:
        out_type = YUV;
        break;
    default:
        out_type = RGB;
        break;
    }

    if (in_type == RGB && out_type == YUV)
        csc_type = RGB_YUV;
    else if (in_type == YUV && out_type == RGB)
        csc_type = YUV_RGB;
    else
        csc_type = NO_CSC;

    /*step2: setup idma channel for background only */
    ipu_disp_bg_idmac_config(ipu_index, mem_addr0, mem_addr1, panel->width, panel->height,
                             mem_colorimetry);
    ipu_dmfc_config(ipu_index, channel);
    ipu_dc_config(ipu_index, channel, di, panel->width, panel->colorimetry);
    ipu_dp_config(ipu_index, csc_type, 0, 0, 0, 0);
    ipu_di_config(ipu_index, di, panel);
}
#endif

#define VIDEO_X1	(pGD->winSizeX)
#define VIDEO_Y1	(pGD->winSizeY)
#define VIDEO_PIX_SIZE1	(pGD->gdfBytesPP)

#define VIDEO_SIZE_GUI1 VIDEO_X1*VIDEO_Y1*VIDEO_PIX_SIZE1

void refresh_screen(uint32_t ipu_index)
{
    // timeout = 50ms
    // int timeout = 500000;
	int i;
	u32 lines = 0;
	int k=0,j=0;
	uint32_t in_buffer = CH23_EBA0;
	uint32_t out_buffer = CH20_EBA0;
	// out_buffer = out_buffer + 600*2 + 227*1920*2;
	// in_buffer = in_buffer + 128 + 32*1920*2;
    // 32*2 + 32*800*2 51264
	// 128 + 32*800*2 51328
	//
	// in_buffer = in_buffer + 123008;
	// out_buffer = out_buffer + 872880;
	// in_buffer = in_buffer + 123008;
	// out_buffer = out_buffer + 192200;
	// in_buffer = in_buffer + 51328;
	//out_buffer = out_buffer + 51264;
	
	if(gic_ipu1_eof_flag == 1) {
	#if 1
		for(lines = 0;lines < 600;lines++)
		{
			YUV422_to_RGB565((u32*)(in_buffer + (800*lines*2)),\
				(u32*)(out_buffer + (800*lines*2)), 1440);
		}
	#endif
		// while (ipu_idmac_channel_busy(1, 23)); // wait display fb idle
		// memcpy((void *)(pGD->frameAdrs), (void *)CH20_EBA0, 1920 * 1080 * 2);

		// memcpy((void *)((pGD->frameAdrs) + 872880), (void *)out_buffer, 2211840);
		memcpy((void *)((pGD->frameAdrs) ), (void *)out_buffer, 768000);
		// memcpy((void *)((pGD->frameAdrs) + 51328), (void *)out_buffer, 480000);
		flush_cache(pGD->frameAdrs, VIDEO_SIZE_GUI1);		
		// buffer ready
	    ipu_channel_buf_ready(ipu_index, 0, 0);
	    
	}
	
	
}


void test_eof(uint32_t ipu_index)
{
    // timeout = 50ms
    int timeout = 5000;
	u32* in_buffer = CH23_EBA0;
	u32* out_buffer = CH20_EBA0;
// test mode rgb
	// ipu_write_field(ipu_index, IPU_CSI0_TST_CTRL__CSI0_PG_B_VALUE, 0xaa);
	// ipu_write_field(ipu_index, IPU_CSI0_TST_CTRL__CSI0_PG_G_VALUE, 0xaa);
	// ipu_write_field(ipu_index, IPU_CSI0_TST_CTRL__CSI0_PG_R_VALUE, 0xaa);

    /* enable idmac channel 0 eof and wait for eof */
    ipu_write_field(ipu_index, IPU_IPU_INT_CTRL_1__IDMAC_EOF_EN_0, 1);
    // clear EOF intterupt
    ipu_write_field(ipu_index, IPU_IPU_INT_STAT_1__IDMAC_EOF_0, 1);

    // force EOF
    // ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_FORCE_EOF,1);
    // read idmac_EOF_0
    while (!(ipu_read(ipu_index, IPU_IPU_INT_STAT_1__ADDR) & 0x1)) {
        udelay(10);
        if (timeout <= 0) {
			printf("eof time out on eof event.\n");
			break;
        }            
        timeout--;
    }

	YUV422_to_RGB565((u32*)(in_buffer),(u32*)(out_buffer), 1920 * 1080 * 2);
	memcpy((void *)(pGD->frameAdrs), (void *)CH20_EBA0, 1920 * 1080 * 2);

}

void ipu_capture_streamoff(uint32_t ipu_index)
{
    int timeout = 5000;

    /*wait for idmac eof and disable csi-->smfc-->idmac */

    /* enable idmac channel 0 eof and wait for eof */
    ipu_write_field(ipu_index, IPU_IPU_INT_CTRL_1__IDMAC_EOF_EN_0, 1);
    ipu_write_field(ipu_index, IPU_IPU_INT_STAT_1__IDMAC_EOF_0, 1);

    while (!(ipu_read(ipu_index, IPU_IPU_INT_STAT_1__ADDR) & 0x1)) {
        udelay(10);
        if (timeout <= 0)
            break;
        timeout--;
    }

    ipu_disable_csi(ipu_index, 0);
    ipu_disable_smfc(ipu_index);

    ipu_idmac_channel_enable(ipu_index, CSI_TO_MEM_CH0, 0);
}

void ipu_mipi_csi2_setup(uint32_t ipu_index, uint32_t csi_width, uint32_t csi_height,
                         ips_dev_panel_t * panel)
{
    uint32_t csi_in_channel = CSI_TO_MEM_CH0;
    ipu_idmac_info_t idmac_info;

    /*step1: config the csi: idma channel (csi -- mem), smfc, csi */

    /*setup idma background channel from MEM to display
       channel: 23
     */
    memset(&idmac_info, 0, sizeof(ipu_idmac_info_t));
    idmac_info.channel = csi_in_channel;
    idmac_info.addr0 = CH23_EBA0;
    idmac_info.addr1 = CH23_EBA1;
    idmac_info.width = csi_width;
    idmac_info.height = csi_height;
    idmac_info.pixel_format = PARTIAL_INTERLEAVED_YUV420;
    idmac_info.sl = panel->width;
    idmac_info.u_offset = panel->width * panel->height;
    idmac_info.npb = 15;
    ipu_general_idmac_config(ipu_index, &idmac_info);

    /*step2: allocate smfc fifo for CSI input channel */
    ipu_smfc_fifo_allocate(ipu_index, 0, 0, 3);

    /*step3: config csi for IPU */
    ipu_csi_config(ipu_index, CSI_MIPI, csi_width, csi_height, csi_width, csi_height);

    /*step4: config display channel: idma, dmfc, dc, dp, di */
    ipu_display_setup(ipu_index, CH23_EBA0, CH23_EBA1, PARTIAL_INTERLEAVED_YUV420, panel);

    /*step5: link csi and display */
    ipu_capture_disp_link(ipu_index, 0);

    /*step6: paint the other display area to white. */
    memset((void *)CH23_EBA0, 0xFF, 2 * panel->width * panel->height);
    memset((void *)(CH23_EBA0 + panel->width * panel->height), 0x80,
           panel->width * panel->height / 2);
    memset((void *)CH23_EBA1, 0xFF, 2 * panel->width * panel->height);
    memset((void *)(CH23_EBA1 + panel->width * panel->height), 0x80,
           panel->width * panel->height / 2);
}

void ipu1_ch0_eobnd_isr(void)
{
    static int i = 0;
    static int frame = 0;
    int frame_width = 1024;
    int cap_height = 480; 
    int band_height = 128; // must match with DMA settings
    int band_num = (cap_height + band_height - 1) / band_height;

    ipu_write_field(1, IPU_IPU_INT_STAT_11__IDMAC_EOBND_0, 1); // clear interrupt state
    
    // copy out the data before it was overwritten, data in YUV444 format
    if(i%band_num != band_num - 1)
        memcpy((void *)(CH27_EBA0 + frame_width*band_height*2*(i%band_num)), (void *)CH23_EBA0, frame_width*band_height*2);
    else
        memcpy((void *)(CH27_EBA0 + frame_width*band_height*2*(i%band_num)), (void *)CH23_EBA0, frame_width*(cap_height - band_height*(band_num-1))*2);

    if((ipu_read(1, IPU_IPU_INT_STAT_1__ADDR) & 0x1) == 1) // clear interrupt state
    {
        ipu_write_field(1, IPU_IPU_INT_STAT_1__IDMAC_EOF_0, 1);
        frame ++;
    }
    
    i++;
}

void ipu_ch0_eobnd_interrupt_register(int ipu_index)
{
    int irq_id = IPU1_SYNC + (ipu_index - 1) * 2;

    // register the IRQ routine
    // register_interrupt_routine(irq_id, &ipu1_ch0_eobnd_isr);

    memset((void *)CH27_EBA0, 0xFF, 1024*768*2);
    memset((void *)CH23_EBA0, 0xFF, 1024*768*2);

    // enable interrupt
    ipu_write_field(ipu_index, IPU_IPU_INT_CTRL_1__IDMAC_EOF_EN_0, 1); // enable band mode
    ipu_write_field(ipu_index, IPU_IPU_INT_CTRL_11__IDMAC_EOBND_EN_0, 1); // enable band mode
   // enable_interrupt(irq_id, CPU_0, 0);
}

void ipu_reg_test(uint32_t ipu_index)
{

    uint32_t temp;
	uint32_t rdata;

	// rdata = __raw_readl(0x02a00200);
	// printf("IPU_IPU_INT_STAT_1__ADDR = %x",rdata);

	// ipu_channel_buf_ready(ipu_index, 0, 0);
	//  temp = ipu_read(ipu_index, IPU_IPU_INT_STAT_1__ADDR);
	temp = IPU_IPU_INT_STAT_1__ADDR + REGS_IPU_BASE(ipu_index);
	printf("IPU_IPU_INT_STAT_1__ADDR = %x",temp);

	// ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_DATA_DEST, 4);


}


void ipu_capture_setup(uint32_t ipu_index, uint32_t csi_interface, uint32_t raw_width,
                       uint32_t raw_height, uint32_t act_width, uint32_t act_height,
                       ips_dev_panel_t * panel)
{
    uint32_t csi_in_channel = CSI_TO_MEM_CH0;
	uint32_t disp_channel = MEM_TO_DP_BG_CH23;
    ipu_idmac_info_t idmac_info;
	// uint32_t csi_mem0 = (uint32_t)(pGD->frameAdrs);
	// uint32_t csi_mem1 = 0;
    uint32_t csi_mem0 = CH23_EBA0, csi_mem1 = 0;
    uint32_t disp_mem0 = csi_mem0, disp_mem1 = csi_mem1;
    uint32_t csi_pixel_format = INTERLEAVED_U1Y1V1Y2;
	ic_csc_params_t csc_params;
    ipu_res_info_t res_info;
	uint32_t temp_mem0 = CH20_EBA0;
	uint32_t temp_i = 0;
    
    // csi_interface = CSI_PARALLEL;
    // csi_vdi_direct_path = 0;  // no deinterlace
    
    /*step1: config the csi: idma channel (csi -- mem), smfc, csi */
    memset(&idmac_info, 0, sizeof(ipu_idmac_info_t));
    idmac_info.addr1 = csi_mem1;
	// act_width used in csi config
   // idmac_info.width = act_width;    //720
   // idmac_info.height = act_height;  //576
    idmac_info.width = raw_width;    // 804
    idmac_info.height = raw_height;  // 625
    idmac_info.pixel_format = csi_pixel_format;  //YUV422

    if (csi_interface == CSI_BT656_NTSC_INTERLACED || csi_interface == CSI_BT656_PAL_INTERLACED)
        idmac_info.so = 1;
    else
        idmac_info.so = 0;

    if ((csi_pixel_format & 0xF) >= INTERLEAVED_RGB) {
        idmac_info.sl = panel->width * 2;
        idmac_info.u_offset = 0;
    } else {
        idmac_info.sl = panel->width;
        idmac_info.u_offset = panel->width * panel->height;
    }
    idmac_info.npb = 15;

	    /*step1: paint the other display area to white. */
#if 1
    // set back color is white
     memset((void *)disp_mem0, 0xFF, panel->width * panel->height * 2);
     memset((void *)temp_mem0, 0x34, panel->width * panel->height * 2);
	 while (ipu_idmac_channel_busy(1, 23)); // wait display fb idle
	 memcpy((void *)((pGD->frameAdrs)), (void *)temp_mem0, panel->width * panel->height * 2);
	 video_logo();
	 while (ipu_idmac_channel_busy(1, 23)); // wait display fb idle
	 memcpy((void *)((pGD->frameAdrs)), (void *)temp_mem0, panel->width * panel->height * 2);
	 video_logo();
	 while (ipu_idmac_channel_busy(1, 23)); // wait display fb idle
	 memcpy((void *)((pGD->frameAdrs)), (void *)temp_mem0, panel->width * panel->height * 2);
	 video_logo();
#endif

	/*step3: config csi for IPU */
	// data dest is IC or SMFC
	// ipu_csi_config(1, csi_interface, raw_width, raw_height, act_width, act_height);
       ipu_csi_config(ipu_index, csi_interface, raw_width, raw_height, raw_width, raw_height);

// step 2: config idmac smfc or vdi ic
#if 1
	 /* setup destination */
	//destination is IDMAC via smfc	
	ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_DATA_DEST, 4); 	
    
	idmac_info.channel = CSI_TO_MEM_CH0;
	idmac_info.addr0 = csi_mem0;
    // flow1 when did not use deinterlace
    // CSI --> SMFC --> IDMAC --> CH0_EBA0
    // non interlaced
	// config idmac
       ipu_general_idmac_config(ipu_index, &idmac_info);
     /*step2: allocate smfc fifo for CSI input channel */
     // ipu_index, 0, 0, 3 ????
     // channel = 0, map = 0, burst size = 3
     // PFS = 8(INTERLEAVED_Y1U1Y2V1  YUV422), 
     // BPP = 3(16bit), NPB=15(16p per burst)
       ipu_smfc_fifo_allocate(ipu_index, 0, 0, 3);
#endif

// csc color conversion
#if 0

		/*setup ic main processing task channel for CSC */
		memset(&res_info, 0x00, sizeof(ipu_res_info_t));
		res_info.addr0_in = CH23_EBA0;
		res_info.pixel_format_in = INTERLEAVED_U1Y1V1Y2;
		res_info.width_in = raw_width;  // 864
		res_info.height_in = raw_height;  // 625
		res_info.strideline_in = panel->width * 2;  // 1920 * 2
		res_info.addr0_out = CH20_EBA0;
		res_info.pixel_format_out = INTERLEAVED_RGB565;
		res_info.width_out = raw_width;
		res_info.height_out = raw_height;
		res_info.strideline_out = panel->width * 2;
		res_info.u_offset_out = 0;  // ubo value
		// confi in and out mem in idmac
		ipu_resize_idmac_config(ipu_index, 11, 22, res_info);

		// ic input sel csi.
       // ipu_write_field(ipu_index, IPU_IPU_CONF__IC_INPUT, 0);
		// CSI destination is IC, IC input sel is CSI
        // ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_DATA_DEST, 2);
		// ic ch21 to IDMAC
	   // ipu_write_field(ipu_index, IPU_IPU_CONF__IC_DMFC_SEL, 0);

	    // CSI_MEM_WR_EN=0, RWS_EN=1 - data is fed from the CSI 
	    // to the system memory (via the IC) and from
        // the system memory to the IC for processing;
      //  ipu_write_field(ipu_index, IPU_IC_CONF__RWS_EN, 1);
      //  ipu_write_field(ipu_index, IPU_IC_CONF__CSI_MEM_WR_EN, 0);

        // config color conversion
            //set ic task
        ipu_ic_enable(ipu_index, 0, 0);  //disable ic
        ipu_ic_resize_config(ipu_index, PP_TASK, res_info);
		
		memset(&csc_params, 0x00, sizeof(ic_csc_params_t));
	    csc_params.taskType = PP_TASK;
	    csc_params.inFormat = YUV;
	    csc_params.outFormat = RGB;
        // set idmac channel for ic csc
        // 2 is tpm addr offset
	    ipu_ic_csc_config(ipu_index, 1, csc_params);

		    //enable ic task
        // task name as ic_task_type enum
        // preprocessing for viewfinder, preprocessing view
        ipu_ic_task_enable(ipu_index, PP_TASK, IC_PP, 1);
		ipu_ic_enable(ipu_index, 1, 0); // enable ic

#endif

// vdi de-interlaced
#if 0
		idmac_info.channel = VDI_TO_MEM_CH13;
		// vdi idmac config when use deinterlaced
		// CSI --> vdi --> ic --> IDMAC --> EBA23
		// CSI destination is IC, IC input sel is vdi
		ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_DATA_DEST, 2); 
		ips_deinterlace_proc(act_width, act_height, panel);		 
#endif

#if 1

//	Show_Str(60,20,200,16,"西安集方电子科技有限公司");
	LCD_False_GBK(60,20);	
    // interrupt init
    gic_ipu_init();

  	/* enable idmac channel 0 eof interrupt */
  	ipu_write_field(ipu_index, IPU_IPU_INT_CTRL_1__IDMAC_EOF_EN_0, 1);

    // ipu interrupt test
  	gic_ipu_test();

    // buffer ready
	ipu_channel_buf_ready(ipu_index, 0, 0);
	
    // while(1)
    for (temp_i = 0; temp_i < 500;temp_i++)
    {
		// ipu_channel_buf_ready(ipu_index, 0, 1);  
		// while (ipu_idmac_channel_busy(1, 0));

	    refresh_screen(ipu_index);
	    LCD_Straight_Line((pGD->winSizeX)/2,(pGD->winSizeY)/2,40);
    }

#endif
  //  while (ipu_idmac_channel_busy(1, 0)) ;

  //  memcpy((void *)(pGD->frameAdrs), (void *)CH23_EBA0, 1920 * 1080 * 2);

 #if 0
 
	ipu_channel_buf_ready(ipu_index, 0, 0);
    udelay(2000000); // 1s

    ipu_channel_buf_ready(ipu_index, 11, 0);
    ipu_channel_buf_ready(ipu_index, 22, 0);
	while (ipu_idmac_channel_busy(1, 0)) ;
	while (ipu_idmac_channel_busy(1, 11)) ;
    while (ipu_idmac_channel_busy(1, 22)) ;

	test_eof(1);

		ipu_channel_buf_ready(ipu_index, 0, 0);
    udelay(2000000); // 1s

    ipu_channel_buf_ready(ipu_index, 11, 0);
    ipu_channel_buf_ready(ipu_index, 22, 0);
	while (ipu_idmac_channel_busy(1, 0)) ;
	while (ipu_idmac_channel_busy(1, 11)) ;
    while (ipu_idmac_channel_busy(1, 22)) ;


	test_eof(1);


	
#endif

}

