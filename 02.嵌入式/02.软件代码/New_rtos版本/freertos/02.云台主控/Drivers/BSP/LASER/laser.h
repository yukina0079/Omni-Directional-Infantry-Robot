
#ifndef __LASER_H
#define __LASER_H

#include "sys.h"


/******************************************************************************************/
/* 引脚 定义 */


#define LASER1_GPIO_PORT                  GPIOC
#define LASER1_GPIO_PIN                   GPIO_PIN_8
#define LASER1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)             /* PF口时钟使能 */

#define LASER2_GPIO_PORT                  GPIOC
#define LASER2_GPIO_PIN                   GPIO_PIN_9
#define LASER2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)             /* PF口时钟使能 */

/******************************************************************************************/

/* LED端口定义 */
#define LASER1(x)   do{ x ? \
                      HAL_GPIO_WritePin(LASER1_GPIO_PORT, LASER1_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LASER1_GPIO_PORT, LASER1_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)       /* LED0 = RED */

#define LASER2(x)   do{ x ? \
                      HAL_GPIO_WritePin(LASER2_GPIO_PORT, LASER2_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LASER2_GPIO_PORT, LASER2_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)       /* LED1 = GREEN */

/******************************************************************************************/
/* 外部接口函数*/
void laser_init(void);                                                                            /* 初始化 */

#endif
