


#ifndef __IPU_CSI_H__
#define __IPU_CSI_H__

#include <linux/types.h>



void ipu_csi_config(uint32_t ipu_index, uint32_t csi_interface, uint32_t raw_width,
                    uint32_t raw_height, uint32_t act_width, uint32_t act_height);


void ipu_capture_disp_link(uint32_t ipu_index, uint32_t smfc);

void ipu_disable_csi(uint32_t ipu_index, uint32_t csi);

uint32_t ipu_smfc_fifo_allocate(uint32_t ipu_index, uint32_t channel, uint32_t map,
                                uint32_t burst_size);

void ipu_disable_smfc(uint32_t ipu_index);




#endif



