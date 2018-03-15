


#ifndef __IPU_IDMAC_H__
#define __IPU_IDMAC_H__

#include <linux/types.h>

void ipu_cpmem_set_field(uint32_t base, int32_t w, int32_t bit, int32_t size, uint32_t v);

void ipu_cpmem_mod_field(uint32_t base, int32_t w, int32_t bit, int32_t size, uint32_t v);

uint32_t ipu_cpmem_read_field(uint32_t base, int32_t w, int32_t bit, int32_t size);

void ipu_general_idmac_config(uint32_t ipu_index, ipu_idmac_info_t * idmac_info);

void ipu_idma_pixel_format_config(uint32_t ipu_index, uint32_t channel, uint32_t pixel_format,
                                  uint32_t so, uint32_t sl, uint32_t ubo);

void ipu_channel_buf_ready(int32_t ipu_index, int32_t channel, int32_t buf);

void ipu_deinterlace_idmac_config(uint32_t ipu_index, uint32_t channel_in, uint32_t channel_out,
                                  ipu_vdi_info_t res_info);

int32_t ipu_idmac_channel_busy(int32_t ipu_index, int32_t channel);


void ipu_disp_bg_idmac_config(uint32_t ipu_index, uint32_t addr0, uint32_t addr1, uint32_t width,
								  uint32_t height, uint32_t pixel_format);


#endif








