

#ifndef __ADV7280_DEF_H__
#define __ADV7280_DEF_H__

#include <linux/types.h>




//! @brief ADV7180 video signal mode
typedef enum {
	ADV7180_NTSC = 0,	//!< Locked on (M) NTSC video signal.
	ADV7180_PAL = 1,	//!< (B, G, H, I, N)PAL video signal.
	ADV7180_NOT_LOCKED,	//!< Not locked on a signal.
} video_fmt_idx;

//! @brief Video format structure.
typedef struct {
	int32_t fmt_id;			//!< Video for linux ID.
	char name[16];			//!< Name (e.g., "NTSC", "PAL", etc.)
	uint16_t raw_width;		//!< Raw width.
	uint16_t raw_height;	//!< Raw height.
	uint16_t active_width;	//!< Active width.
	uint16_t active_height;	//!< Active height.
} video_fmt_t;

#endif

