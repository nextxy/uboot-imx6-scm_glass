/*
 * Copyright (c) 2011-2012, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file ips_disp_panel.c
 * @brief IPU Software library, display panel parameters setting and initialization
 * @ingroup diag_ipu
 */

#include <common.h>
#include <linux/types.h>
#include <linux/err.h>
#include <asm/io.h>

#include "ipu_common.h"
#include "ips_disp_panel.h"


ips_dev_panel_t disp_dev_list[] = {
    {
     "HDMI 1080P 60Hz",         // name
     HDMI_1080P60,              // panel id flag
     DISP_DEV_HDMI,                   // panel type
     DCMAP_RGB888,              // data format for panel
     60,                        // refresh rate
     1024,                      // panel width
     768,                      //panel height
     148500000,                 // pixel clock frequency
     192,                       // hsync start width
     44,                        // hsync width
     88,                        // hsyn back width
     41,                        // vysnc start width
     5,                         // vsync width
     4,                         // vsync back width
     0,                         // delay from hsync to vsync
     0,                         // interlaced mode
     1,                         // clock selection, external
     0,                         // clock polarity
     1,                         // hsync polarity
     1,                         // vync polarity
     1,                         // drdy polarity
     0,                         // data polarity
     }
};

uint32_t num_of_panels = sizeof(disp_dev_list) / sizeof(ips_dev_panel_t);

/*! Set display parameters in IPU configuration structure according to your display panel name. There are only some displays are supported by this function. And you can set the display manually all by your self if the hardware is supported by IPU.
 *
 * @param panel_name 		panel name of your display
 */
ips_dev_panel_t *search_panel(char *panel_name)
{
    ips_dev_panel_t *panel = &disp_dev_list[0];
    int32_t index = 0;

    while (index < num_of_panels) {
        if (!strcmp(panel->panel_name, panel_name))
            break;
        else {
            panel++;
            index++;
        }
    }

    if (index == num_of_panels) {
        printf("The display panel %s is not supported!\n", panel_name);
        return NULL;
    }

    return panel;
}


/*! @brief list the supported panel of specific type.
 *
 * @param panel_type 		panel type of display
 */
void list_panel(uint32_t panel_type)
{
    ips_dev_panel_t *panel = &disp_dev_list[0];
    int32_t index = 0;

    while (index < num_of_panels) {
        if (panel->panel_type == panel_type)
        {
            printf("\t%d : %s\n", panel->panel_id, panel->panel_name);
        }
        panel++;
        index++;
    }
}

ips_dev_panel_t *get_panel_by_id(uint32_t panel_id)
{
    ips_dev_panel_t *panel = &disp_dev_list[0];
    int32_t index = 0;

    while (index < num_of_panels) {
        if (panel->panel_id == panel_id)
        {
            return panel;
        }
        panel++;
        index++;
    }

    return NULL;
}
