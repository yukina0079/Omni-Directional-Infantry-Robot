#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"
#include "./SYSTEM/sys/sys.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{

  while (1)
  {
  }
}

void MemManage_Handler(void)
{

  while (1)
  {
  }
}

void BusFault_Handler(void)
{

  while (1)
  {
  }
}

void UsageFault_Handler(void)
{

  while (1)
  {
  }
}

#if (!SYS_SUPPORT_OS)
void SVC_Handler(void)
{
}
#endif

void DebugMon_Handler(void)
{
}

#if (!SYS_SUPPORT_OS)
void PendSV_Handler(void)
{
}
#endif

#if (!SYS_SUPPORT_OS)
void SysTick_Handler(void)
{
  HAL_IncTick();
}
#endif
