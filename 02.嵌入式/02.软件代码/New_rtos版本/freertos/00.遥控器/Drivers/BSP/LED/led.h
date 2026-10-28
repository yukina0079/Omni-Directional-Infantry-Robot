#ifndef __LED_H
#define __LED_H

#include "sys.h"

#define LED0_GPIO_PORT          GPIOA
#define LED0_GPIO_PIN           GPIO_PIN_8
#define LED0_GPIO_CLK_ENABLE()  do { __HAL_RCC_GPIOA_CLK_ENABLE(); } while (0)
#define LED1_GPIO_PORT          GPIOA
#define LED1_GPIO_PIN           GPIO_PIN_12
#define LED1_GPIO_CLK_ENABLE()  do { __HAL_RCC_GPIOA_CLK_ENABLE(); } while (0)
#define LED2_GPIO_PORT          GPIOB
#define LED2_GPIO_PIN           GPIO_PIN_1
#define LED2_GPIO_CLK_ENABLE()  do { __HAL_RCC_GPIOB_CLK_ENABLE(); } while (0)

#define LED0(x)                 do { (x) ?                                                              \
                                    HAL_GPIO_WritePin(LED0_GPIO_PORT, LED0_GPIO_PIN, GPIO_PIN_SET):     \
                                    HAL_GPIO_WritePin(LED0_GPIO_PORT, LED0_GPIO_PIN, GPIO_PIN_RESET);   \
                                } while (0)
#define LED1(x)                 do { (x) ?                                                              \
                                    HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET):     \
                                    HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET);   \
                                } while (0)

#define LED0_TOGGLE()           do { HAL_GPIO_TogglePin(LED0_GPIO_PORT, LED0_GPIO_PIN); } while (0)
#define LED1_TOGGLE()           do { HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN); } while (0)
#define LED2_TOGGLE()           do { HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_GPIO_PIN); } while (0)

void led_init(void);

#endif
