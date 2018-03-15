


#ifndef __IPU_DMFC_H__
#define __IPU_DMFC_H__

#include <linux/types.h>


void ipu_dmfc_config(uint32_t ipu_index, uint32_t channel);

void ipu_dmfc_alloc(uint32_t ipu_index, uint32_t channel, uint32_t size, uint32_t start_addr,
                    uint32_t burst);

void ipu_resize_idmac_config(uint32_t ipu_index, uint32_t channel_in, uint32_t channel_out,
                             ipu_res_info_t res_info);

void ipu_idmac_channel_enable(int32_t ipu_index, int32_t channel, int32_t enable);


#endif





