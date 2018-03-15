
#ifndef __IPS_DISP_PANEL_H__
#define __IPS_DISP_PANEL_H__

#include <linux/types.h>


void list_panel(uint32_t panel_type);

ips_dev_panel_t *get_panel_by_id(uint32_t panel_id);

ips_dev_panel_t *search_panel(char *panel_name);


#endif



