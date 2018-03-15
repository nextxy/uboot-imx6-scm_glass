


#ifndef __IPU_DC_H__
#define __IPU_DC_H__

#include <linux/types.h>


void ipu_dc_config(uint32_t ipu_index, uint32_t channel, uint32_t di, uint32_t width,
                   uint32_t colorimetry);


int32_t ipu_dc_write_channel_config(int32_t ipu_index, int32_t dma_channel, int32_t disp_port,
                                    int32_t link_di_index, int32_t field_mode_enable);


int32_t ipu_dc_display_config(int32_t ipu_index, int32_t display_port, int32_t type,
                              int32_t increment, int32_t strideline);

int32_t ipu_dc_map(int32_t ipu_index, int32_t map, int32_t format);


void ipu_dc_microcode_config(int32_t ipu_index, dc_microcode_t microcode);

void ipu_dc_microcode_event(int32_t ipu_index, int32_t channel, int32_t event, int32_t priority,
                            int32_t address);



#endif









