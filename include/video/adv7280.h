

#ifndef __CORE_ADV7280_H_
#define __CORE_ADV7280_H_

#include <i2c.h>

/*************************************************/
int adv7280_init(unsigned char i2cbus);
int adv7280_reg_read(unsigned char i2cbus, u32 reg, u32 *val);
int adv7280_reg_write(unsigned char i2cbus, u32 reg, u32 val);
int32_t adv7280_capture(unsigned char i2cbus);


#endif /* __CORE_PMIC_H_ */

