



#ifndef __IPU_IC_H__
#define __IPU_IC_H__

#include <linux/types.h>


void ipu_ic_enable(int32_t ipu_index, int32_t ic_enable, int32_t irt_enable);

void ipu_ic_deinterlace_config(int32_t ipu_index, ipu_vdi_info_t vdi_info);

int32_t ipu_ic_task_enable(int32_t ipu_index, int32_t task_type, int32_t task, int32_t enable);

int32_t ipu_ic_calc_resize_coeffs(int32_t in_size, int32_t out_size, int32_t * resize_coeff,
                                  int32_t * downsize_coeff);
void ipu_ic_rotation_config(int32_t ipu_index, int32_t taskType, int32_t rot, int32_t hf,
                            int32_t vf);
void ipu_ic_resize_config(int32_t ipu_index, int32_t taskType, ipu_res_info_t res_info);

int32_t ipu_ic_config_resize_rate(int32_t ipu_index, char *task_type, uint32_t res_vert,
                                  uint32_t down_vert, uint32_t res_horiz, uint32_t down_horiz);

int32_t ipu_ic_combine_config(int32_t ipu_index, ic_comb_params_t comb_params);

int32_t ipu_ic_csc_config(int32_t ipu_index, int32_t csc_set_index, ic_csc_params_t csc_params);
int32_t ipu_ic_task_enable(int32_t ipu_index, int32_t task_type, int32_t task, int32_t enable);





#endif





