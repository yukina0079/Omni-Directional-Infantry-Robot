#include "./SYSTEM/sys/sys.h"

void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset)
{

    SCB->VTOR = baseaddr | (offset & (uint32_t)0xFFFFFE00);
}

void sys_wfi_set(void)
{
    __ASM volatile("wfi");
}

void sys_intx_disable(void)
{
    __ASM volatile("cpsid i");
}

void sys_intx_enable(void)
{
    __ASM volatile("cpsie i");
}

void sys_msr_msp(uint32_t addr)
{
    __set_MSP(addr);
}

void sys_standby(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    SET_BIT(PWR->CR, PWR_CR_PDDS);
}

void sys_soft_reset(void)
{
    NVIC_SystemReset();
}

uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)
{
    HAL_StatusTypeDef ret = HAL_OK;
    RCC_ClkInitTypeDef rcc_clk_init_handle;
    RCC_OscInitTypeDef rcc_osc_init_handle;

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    rcc_osc_init_handle.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    rcc_osc_init_handle.HSEState = RCC_HSE_ON;
    rcc_osc_init_handle.PLL.PLLState = RCC_PLL_ON;
    rcc_osc_init_handle.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    rcc_osc_init_handle.PLL.PLLN = plln;
    rcc_osc_init_handle.PLL.PLLM = pllm;
    rcc_osc_init_handle.PLL.PLLP = pllp;
    rcc_osc_init_handle.PLL.PLLQ = pllq;

    ret=HAL_RCC_OscConfig(&rcc_osc_init_handle);
    if(ret != HAL_OK)
    {
        return 1;
    }

    rcc_clk_init_handle.ClockType = ( RCC_CLOCKTYPE_SYSCLK \
                                    | RCC_CLOCKTYPE_HCLK \
                                    | RCC_CLOCKTYPE_PCLK1 \
                                    | RCC_CLOCKTYPE_PCLK2);

    rcc_clk_init_handle.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk_init_handle.AHBCLKDivider = RCC_SYSCLK_DIV1;
    rcc_clk_init_handle.APB1CLKDivider = RCC_HCLK_DIV4;
    rcc_clk_init_handle.APB2CLKDivider = RCC_HCLK_DIV2;

    ret = HAL_RCC_ClockConfig(&rcc_clk_init_handle, FLASH_LATENCY_4);
    if(ret != HAL_OK)
    {
        return 1;
    }

    if (HAL_GetREVID() == 0x1001)
    {
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }
    return 0;
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
    while (1)
    {
    }
}
#endif
