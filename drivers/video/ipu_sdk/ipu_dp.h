


#ifndef __IPU_DP_H__
#define __IPU_DP_H__

#include <linux/types.h>


void ipu_dp_config(uint32_t ipu_index, uint32_t csc_type, uint32_t dual_disp, uint32_t fg_xp,
                   uint32_t fg_yp, uint32_t alpha);


void ipu_dp_fg_config(uint32_t ipu_index, uint32_t dual_disp, uint32_t fg_xp, uint32_t fg_yp,
                      uint32_t alpha);

void ipu_dp_csc_config(uint32_t ipu_index, uint32_t dp, uint32_t csc_type);


#endif





