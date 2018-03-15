




#ifndef __IPU_DI_H__
#define __IPU_DI_H__

#include <linux/types.h>


void ipu_di_config(uint32_t ipu_index, uint32_t di, ips_dev_panel_t * panel);


void ipu_di_waveform_config(int32_t ipu_index, int32_t di, int32_t pointer, int32_t set, int32_t up,
                            int32_t down);

int32_t ipu_di_screen_set(int32_t ipu_index, int32_t di, int32_t screen_height);


void ipu_di_pointer_config(int32_t ipu_index, int32_t di, int32_t pointer, int32_t access,
                           int32_t component, int32_t cst, int32_t * pt);


void ipu_di_sync_config(int32_t ipu_index, int32_t di, int32_t pointer,
                        di_sync_wave_gen_t sync_waveform_gen);


void ipu_di_interface_set(uint32_t ipu_index, uint32_t di, ips_dev_panel_t * panel,
                          uint32_t line_prediction, uint32_t vsync_sel, uint32_t hsync_sel);

int32_t ipu_di_bsclk_gen(int32_t ipu_index, int32_t di, int32_t division, int32_t up, int32_t down);


#endif






