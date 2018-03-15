


#include <common.h>
#include <malloc.h>

#include "ipu_common.h"
#include "ipu_csi.h"


/*
 * set the CSI module to handle data from camera
 * @param width frame width of camera input
 * @param height frame height of camera input
 */
void ipu_csi_config(uint32_t ipu_index, uint32_t csi_interface, uint32_t raw_width,
                    uint32_t raw_height, uint32_t act_width, uint32_t act_height)
{
    int hsync_pol = 0, vsync_pol = 0, clock_mode = 0, data_fmt;
    int hsc = 0, vsc = 0;

    if (csi_interface == CSI_PARALLEL) {
        clock_mode = CSI_CLK_MODE_GATED_CLK;
        data_fmt = CSI_YUYV;
		hsync_pol = 0;
		vsync_pol = 1;
		hsc = 0; // PAL skip pixel
		vsc = 0;
    } else if (csi_interface == CSI_MIPI) {
        clock_mode = CSI_CLK_MODE_NONGATED_CLK;
        data_fmt = CSI_UYVY;
    } else if (csi_interface == CSI_BT656_NTSC_INTERLACED) {
        hsync_pol = 1;
        vsync_pol = 0;
        hsc = 0;
        vsc = 0xD;
        clock_mode = CSI_CLK_MODE_BT656_INTERLACED;
        data_fmt = CSI_UYVY;
    } else if (csi_interface == CSI_BT656_PAL_INTERLACED) {
        hsync_pol = 1;
        vsync_pol = 0;
        hsc = 0;
        vsc = 0x0;
        clock_mode = CSI_CLK_MODE_BT656_INTERLACED;
        data_fmt = CSI_UYVY;
    } else if (csi_interface == CSI_BT656_NTSC_PROGRESSIVE
               || csi_interface == CSI_BT656_PAL_PROGRESSIVE) {
        hsync_pol = 1;
        vsync_pol = 0;
        hsc = 0;
        vsc = 0x0;
        clock_mode = CSI_CLK_MODE_BT656_PROGRESSIVE;
        data_fmt = CSI_UYVY;
    } else if (csi_interface == CSI_TEST_MODE) {
        data_fmt = CSI_YUV444;
        clock_mode = CSI_CLK_MODE_NONGATED_CLK;
        ipu_write_field(ipu_index, IPU_CSI0_TST_CTRL__CSI0_TEST_GEN_MODE, 1);
    } else {
        printf("Unsupport CSI interface\n");
    }

    /* setting CSI data source, default is from parallel interface */
    if (csi_interface == CSI_MIPI) {
        ipu_write_field(ipu_index, IPU_IPU_CONF__CSI0_DATA_SOURCE, 1);  //csi0 data souce is mipi
        switch (data_fmt) {
        case CSI_UYVY:
            ipu_write_field(ipu_index, IPU_CSI0_DI__CSI0_MIPI_DI0, 0x1E);   //(UYVY)MIPI_YUV422 8bit
            break;
        case CSI_RGB565:
            ipu_write_field(ipu_index, IPU_CSI0_DI__CSI0_MIPI_DI0, 0x22);   //MIPI_RGB565
            break;
        case CSI_RGB555:
            ipu_write_field(ipu_index, IPU_CSI0_DI__CSI0_MIPI_DI0, 0x21);   //MIPI_RGB555
            break;
        case CSI_RGB444:
            ipu_write_field(ipu_index, IPU_CSI0_DI__CSI0_MIPI_DI0, 0x20);   //MIPI_RGB565
            break;
        default:
            printf("\nThispixel format is not supported by MIPI_CSI2!\n");
            return;
        }
        /*using CSI0_MIPI_DI0 by default, DI1~3 unused */
        ipu_write_field(ipu_index, IPU_CSI0_DI__CSI0_MIPI_DI1, 0);
        ipu_write_field(ipu_index, IPU_CSI0_DI__CSI0_MIPI_DI2, 0);
        ipu_write_field(ipu_index, IPU_CSI0_DI__CSI0_MIPI_DI3, 0);
    } else {
        /* set parallel interface information */
		// CSI0 Sensor Configuration Register
        ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_DATA_EN_POL, 0); // no invert
        ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_EXT_VSYNC, 1);   // external vs
        ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_SENS_PIX_CLK_POL, 0);   // pos edge
        ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_DATA_POL, 0);    // no invert
		// hsync_pol = 0, active as low
        ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_HSYNC_POL, hsync_pol);
		// vsync_pol = 0, active as low
        ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_VSYNC_POL, vsync_pol);
    }

	// Gated clock mode
    ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_SENS_PRTCL, clock_mode);    
    if(csi_interface == CSI_TEST_MODE) {
		//division ratio of HSP_CLK into SENSOR_MCLK to slow down the frame rate in test mode
        ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_DIV_RATIO, 0x9F);
    }
    else {
		// DIV_RATIO
        ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_DIV_RATIO, 0);
    }
	//8bits per color
    ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_DATA_WIDTH, 1);
	//only when data format is RGB/YUV, and data_width > 8
    ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_PACK_TIGHT, 0); 
	// YUV422, YUYV
    ipu_write_field(ipu_index, IPU_CSI0_SENS_CONF__CSI0_SENS_DATA_FORMAT, data_fmt);
    /*CSI0 common sensor configuration for PApa */
    /*set sensor frame size */
	// CSI0 Sense Frame Size Register
    ipu_write_field(ipu_index, IPU_CSI0_SENS_FRM_SIZE__CSI0_SENS_FRM_HEIGHT, raw_height - 1);
    ipu_write_field(ipu_index, IPU_CSI0_SENS_FRM_SIZE__CSI0_SENS_FRM_WIDTH, raw_width - 1);

    /*CSI_ACT_FRM_SIZE */
	// CSI0 Actual Frame Size Register
    ipu_write_field(ipu_index, IPU_CSI0_ACT_FRM_SIZE__CSI0_ACT_FRM_HEIGHT, act_height - 1);
    ipu_write_field(ipu_index, IPU_CSI0_ACT_FRM_SIZE__CSI0_ACT_FRM_WIDTH, act_width - 1);

    /*CSI_OUT_FRM_CTRL */
	// CSI0 Output Control Register
	// Enable horizontal downsizing = 0
    ipu_write_field(ipu_index, IPU_CSI0_OUT_FRM_CTRL__CSI0_HORZ_DWNS, 0);
	// Enable vertical downsizing = 0
    ipu_write_field(ipu_index, IPU_CSI0_OUT_FRM_CTRL__CSI0_VERT_DWNS, 0);
	// Horizontal skip.
    ipu_write_field(ipu_index, IPU_CSI0_OUT_FRM_CTRL__CSI0_HSC, hsc);
	// Vertical skip.
    ipu_write_field(ipu_index, IPU_CSI0_OUT_FRM_CTRL__CSI0_VSC, vsc);
	
    // Configuration Register (IPU1_CONF)
    ipu_write_field(ipu_index, IPU_IPU_CONF__CSI_SEL, 0); //sel csi0
    ipu_write_field(ipu_index, IPU_IPU_CONF__CSI0_EN, 1); //enable csi0

    // clk mode set by csi_interface
    if (clock_mode == CSI_CLK_MODE_BT656_PROGRESSIVE) {
        ipu_write_field(ipu_index, IPU_CSI0_CCIR_CODE_1__FULL, 0x40030);
        ipu_write_field(ipu_index, IPU_CSI0_CCIR_CODE_3__FULL, 0xFF0000);
    } else if (clock_mode == CSI_CLK_MODE_BT656_INTERLACED) {
        // BT656 code setting
        /*
         * Field0BlankEnd = 0x6 (FVH 0x3), Field0BlankStart = 0x2 (FVH 0x2),
         * Field0ActiveEnd = 0x4 (FVH 0x1), Field0ActiveStart = 0 (FVH 0x0)
         */
        ipu_write_field(ipu_index, IPU_CSI0_CCIR_CODE_1__FULL, 0xD07DF);
        /*
         * Field0BlankEnd = 0x7 (FVH 0x7), Field0BlankStart = 0x3 (FVH 0x6),
         * Field0ActiveEnd = 0x5 (FVH 0x5), Field0ActiveStart = 0x1 (FVH 0x4)
         */
        ipu_write_field(ipu_index, IPU_CSI0_CCIR_CODE_2__FULL, 0x40596);
        ipu_write_field(ipu_index, IPU_CSI0_CCIR_CODE_3__FULL, 0xFF0000);
        ipu_write_field(ipu_index, IPU_CSI0_CCIR_CODE_1__CSI0_CCIR_ERR_DET_EN, 1);
    }
	// printf("CSI init end\n");
}


/*!
 * set the SMFC(fifo for camera input) property
 *
 * @param	ipu_index: ipu index
 * @param 	channel: select the channel number out of 0~3
 * @param 	map: choose the map between channel and fifo
 * @param 	burst_size: set the burst size of fifo input.
 *
 * @return -1 means channel not found, 0 means configuration is ok.
 */
uint32_t ipu_smfc_fifo_allocate(uint32_t ipu_index, uint32_t channel, uint32_t map,
                                uint32_t burst_size)
{
    switch (channel) {
    case 0:
		// channel = 0
		// map CSI frames to IDMAC channels
		// 000 CSI0, ID=0 mapped to DMASMFC channel 0.
        ipu_write_field(ipu_index, IPU_SMFC_MAP__MAP_CH0, map);
		// This register holds the burst size value for each DMASMFC channel.
		// according to pfs,bpp,npb para, burst_size = 3
		// Burst Size of SMFCDMA channel 0.
        ipu_write_field(ipu_index, IPU_SMFC_BS__BURST0_SIZE, burst_size);
        break;

    case 1:
        ipu_write_field(ipu_index, IPU_SMFC_MAP__MAP_CH1, map);
        ipu_write_field(ipu_index, IPU_SMFC_BS__BURST1_SIZE, burst_size);
        break;

    case 2:
        ipu_write_field(ipu_index, IPU_SMFC_MAP__MAP_CH2, map);
        ipu_write_field(ipu_index, IPU_SMFC_BS__BURST2_SIZE, burst_size);
        break;

    case 3:
        ipu_write_field(ipu_index, IPU_SMFC_MAP__MAP_CH3, map);
        ipu_write_field(ipu_index, IPU_SMFC_BS__BURST3_SIZE, burst_size);
        break;

    default:
        printf("Wrong channel selected!!\n");
        return -1;
    }

// enable smfc
    ipu_write_field(ipu_index, IPU_IPU_CONF__SMFC_EN, 1);
    return 0;
}

/*!
 * link CSI channel smfc with display 
 *
 * @param	ipu_index: ipu index
 * @param 	smfc: select the smfc number out of 0/2
 *
 * @return -1 means channel not found, 0 means configuration is ok.
 */
void ipu_capture_disp_link(uint32_t ipu_index, uint32_t smfc)
{
    if (csi_vdi_direct_path == 0) {
        switch (smfc) {
        case 0:
			printf("smfc 0\n");
			// Destination select for SMFC0 = 1001, DP_SYNC0 (ch23)
            ipu_write_field(ipu_index, IPU_IPU_FS_PROC_FLOW3__SMFC0_DEST_SEL, 0x9); // smfc0 -> chan23 DP_SYNC
            // Source select for DS2 - MG (graphics) plane (ch23) = 0001, capture0 (smfc0)
            ipu_write_field(ipu_index, IPU_IPU_FS_DISP_FLOW1__DP_SYNC0_SRC_SEL, 0x1);   // MG source from capture 0
            break;
        case 2:
			printf("smfc 2\n");
            ipu_write_field(ipu_index, IPU_IPU_FS_PROC_FLOW3__SMFC2_DEST_SEL, 0x9); // smfc2 -> chan23
            ipu_write_field(ipu_index, IPU_IPU_FS_DISP_FLOW1__DP_SYNC0_SRC_SEL, 0x2);   // MG source from capture 2
            break;
        default:
            printf("Wrong smfc selected!!\n");
            break;
        }
    } else {
        printf("vdi == 1\n");
        ipu_write_field(ipu_index, IPU_IPU_FS_PROC_FLOW1__VDI_SRC_SEL, 0x01);   // csi
        ipu_write_field(ipu_index, IPU_IPU_FS_PROC_FLOW2__PRPVF_DEST_SEL, 0x09);    // DP sync0     
        ipu_write_field(ipu_index, IPU_IPU_FS_DISP_FLOW1__DP_SYNC0_SRC_SEL, 0x04);  // VF
    }
}

/*!
 * @brief Disable csi module
 *
 * @param	csi csi id
 *
 */
void ipu_disable_csi(uint32_t ipu_index, uint32_t csi)
{
    if (csi == 0) {
        ipu_write_field(ipu_index, IPU_IPU_CONF__CSI0_EN, 0);
    } else if (csi == 1) {
        ipu_write_field(ipu_index, IPU_IPU_CONF__CSI1_EN, 0);
    }
}

/*!
 * @brief Disable smfc module
 *
 */
void ipu_disable_smfc(uint32_t ipu_index)
{
    ipu_write_field(ipu_index, IPU_IPU_CONF__SMFC_EN, 0);
}





