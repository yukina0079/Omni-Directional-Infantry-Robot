#include "spi.h"
#include "nrf24l01.h"
#include "delay.h"
#include "led.h"
#include <stdio.h>

/* A missing radio must not prevent the gimbal controller from starting its
 * FreeRTOS/UART tasks.  The old unbounded probe loop made UART3 appear dead
 * whenever the NRF module was unplugged or still powering up. */
#define NRF_CHECK_MAX_TRIES       100u
#define NRF_CHECK_RETRY_DELAY_MS  10u
#define ADDR_AB_LEN    5   // 涓庡簱鍑芥暟TX_ADR_WIDTH=5涓鑷
#define ADDR_BC_LEN    5   // 涓庡簱鍑芥暟TX_ADR_WIDTH=5涓鑷
#define DATA_WIDTH     13  // 涓庡簱鍑芥暟TX_PLOAD_WIDTH=13涓鑷

// A鈫払 閫氫俊鍙傛暟锛圓銆丅妯″潡鍏辩敤锛
uint8_t ADDR_AB[ADDR_AB_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55};
#define CH_AB          30  // A-B閫氫俊棰戦亾

// B鈫扖 閫氫俊鍙傛暟锛圔銆丆妯″潡鍏辩敤锛
uint8_t ADDR_BC[ADDR_BC_LEN] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
#define CH_BC          40  // B-C閫氫俊棰戦亾
 
extern SPI_HandleTypeDef g_spi2_handler;                /* SPI1鍙ユ焺 */
const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01};    /* 鎺ユ敹鍦板潃 */
const uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01};    /* 鍙戦佸湴鍧 */

/**
 * @brief       閽堝NRF24L01淇敼SPI1椹卞姩
 * @param       鏃
 * @retval      鏃
 */
void nrf24l01_spi_init(void)
{
    __HAL_SPI_DISABLE(&g_spi2_handler);                 /* 鍏堝叧闂璖PI1 */
    g_spi2_handler.Init.CLKPolarity = SPI_POLARITY_LOW; /* 涓茶鍚屾鏃堕挓鐨勭┖闂茬姸鎬佷负浣庣數骞 */
    g_spi2_handler.Init.CLKPhase = SPI_PHASE_1EDGE;     /* 涓茶鍚屾鏃堕挓鐨勭1涓烦鍙樻部锛堜笂鍗囨垨涓嬮檷锛夋暟鎹閲囨牱 */
    HAL_SPI_Init(&g_spi2_handler);
    __HAL_SPI_ENABLE(&g_spi2_handler);                  /* 浣胯兘SPI1 */
}

/**
 * @brief       鍒濆鍖24L01鐨処O鍙
 *   @note      灏哠PI1妯″紡鏀规垚SCK绌洪棽浣庣數骞,鍙奡PI 妯″紡0
 * @param       鏃
 * @retval      鏃
 */
void nrf24l01_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    NRF24L01_CE_GPIO_CLK_ENABLE();  /* CE鑴氭椂閽熶娇鑳 */
    NRF24L01_CSN_GPIO_CLK_ENABLE(); /* CSN鑴氭椂閽熶娇鑳 */
    NRF24L01_IRQ_GPIO_CLK_ENABLE(); /* IRQ鑴氭椂閽熶娇鑳 */

    gpio_init_struct.Pin = NRF24L01_CE_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;             /* 鎺ㄦ尳杈撳嚭 */
    gpio_init_struct.Pull = GPIO_PULLUP;                     /* 涓婃媺 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;           /* 楂橀 */
    HAL_GPIO_Init(NRF24L01_CE_GPIO_PORT, &gpio_init_struct); /* 鍒濆鍖朇E寮曡剼 */

    gpio_init_struct.Pin = NRF24L01_CSN_GPIO_PIN;
    HAL_GPIO_Init(NRF24L01_CSN_GPIO_PORT, &gpio_init_struct);/* 鍒濆鍖朇SN寮曡剼 */

    gpio_init_struct.Pin = NRF24L01_IRQ_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                 /* 杈撳叆 */
    gpio_init_struct.Pull = GPIO_PULLUP;                     /* 涓婃媺 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;           /* 楂橀 */
    HAL_GPIO_Init(NRF24L01_IRQ_GPIO_PORT, &gpio_init_struct);/* 鍒濆鍖朇E寮曡剼 */

    spi2_init();                /* 鍒濆鍖朣PI1 */
    nrf24l01_spi_init();        /* 閽堝NRF鐨勭壒鐐逛慨鏀筍PI鐨勮缃 */
    NRF24L01_CE(0);             /* 浣胯兘24L01 */
    NRF24L01_CSN(1);            /* SPI鐗囬夊彇娑 */
}

/**
 * @brief       妫娴24L01鏄惁瀛樺湪
 * @param       鏃
 * @retval      0, 鎴愬姛; 1, 澶辫触;
 */
uint8_t nrf24l01_check(void)
{
    uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
    uint8_t i;
    
    spi2_set_speed(SPI_SPEED_32);                         /* spi閫熷害涓7.5Mhz锛24L01鐨勬渶澶PI鏃堕挓涓10Mhz锛 */
    nrf24l01_write_buf(NRF_WRITE_REG + TX_ADDR, buf, 5);  /* 鍐欏叆5涓瓧鑺傜殑鍦板潃. */
    nrf24l01_read_buf(TX_ADDR, buf, 5);                   /* 璇诲嚭鍐欏叆鐨勫湴鍧 */

    for (i = 0; i < 5; i++)
    {
        if (buf[i] != 0XA5) break;
    }
    
    if (i != 5) return 1;   /* 妫娴24L01閿欒 */

    return 0;               /* 妫娴嬪埌24L01 */
}

/**
 * @brief       NRF24L01鍐欏瘎瀛樺櫒
 * @param       reg   : 瀵勫瓨鍣ㄥ湴鍧
 * @param       value : 鍐欏叆瀵勫瓨鍣ㄧ殑鍊
 * @retval      鐘舵佸瘎瀛樺櫒鍊
 */
static uint8_t nrf24l01_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    
    NRF24L01_CSN(0);                    /* 浣胯兘SPI浼犺緭 */
    status = spi2_read_write_byte(reg); /* 鍙戦佸瘎瀛樺櫒鍙 */
    spi2_read_write_byte(value);        /* 鍐欏叆瀵勫瓨鍣ㄧ殑鍊 */
    NRF24L01_CSN(1);                    /* 绂佹SPI浼犺緭 */
    
    return status;                      /* 杩斿洖鐘舵佸 */
}

/**
 * @brief       NRF24L01璇诲瘎瀛樺櫒
 * @param       reg   : 瀵勫瓨鍣ㄥ湴鍧
 * @retval      璇诲彇鍒扮殑瀵勫瓨鍣ㄥ;
 */
static uint8_t nrf24l01_read_reg(uint8_t reg)
{
    uint8_t reg_val;
    
    NRF24L01_CSN(0);            /* 浣胯兘SPI浼犺緭 */
    spi2_read_write_byte(reg);  /* 鍙戦佸瘎瀛樺櫒鍙 */
    reg_val = spi2_read_write_byte(0XFF);   /* 璇诲彇瀵勫瓨鍣ㄥ唴瀹 */
    NRF24L01_CSN(1);            /* 绂佹SPI浼犺緭 */
    
    return reg_val;             /* 杩斿洖鐘舵佸 */
}

/**
 * @brief       鍦ㄦ寚瀹氫綅缃鍑烘寚瀹氶暱搴︾殑鏁版嵁
 * @param       reg   : 瀵勫瓨鍣ㄥ湴鍧
 * @param       pbuf  : 鏁版嵁鎸囬拡
 * @param       len   : 鏁版嵁闀垮害
 * @retval      鐘舵佸瘎瀛樺櫒鍊
 */
static uint8_t nrf24l01_read_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
    uint8_t status, i;
    
    NRF24L01_CSN(0);                            /* 浣胯兘SPI浼犺緭 */
    status = spi2_read_write_byte(reg);         /* 鍙戦佸瘎瀛樺櫒鍊(浣嶇疆),骞惰鍙栫姸鎬佸 */

    for (i = 0; i < len; i++)
    {
        pbuf[i] = spi2_read_write_byte(0XFF);   /* 璇诲嚭鏁版嵁 */
    }
    
    NRF24L01_CSN(1);    /* 鍏抽棴SPI浼犺緭 */
    
    return status;      /* 杩斿洖璇诲埌鐨勭姸鎬佸 */
}

/**
 * @brief       鍦ㄦ寚瀹氫綅缃啓鎸囧畾闀垮害鐨勬暟鎹
 * @param       reg   : 瀵勫瓨鍣ㄥ湴鍧
 * @param       pbuf  : 鏁版嵁鎸囬拡
 * @param       len   : 鏁版嵁闀垮害
 * @retval      鐘舵佸瘎瀛樺櫒鍊
 */
static uint8_t nrf24l01_write_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
    uint8_t status, i;
    
    NRF24L01_CSN(0);    /* 浣胯兘SPI浼犺緭 */
    status = spi2_read_write_byte(reg);/* 鍙戦佸瘎瀛樺櫒鍊(浣嶇疆),骞惰鍙栫姸鎬佸 */

    for (i = 0; i < len; i++)
    {
        spi2_read_write_byte(*pbuf++); /* 鍐欏叆鏁版嵁 */
    }
    
    NRF24L01_CSN(1);    /* 鍏抽棴SPI浼犺緭 */
    
    return status;      /* 杩斿洖璇诲埌鐨勭姸鎬佸 */
}

/**
 * @brief       鍚姩NRF24L01鍙戦佷竴娆℃暟鎹(鏁版嵁闀垮害 = TX_PLOAD_WIDTH)
 * @param       ptxbuf : 寰呭彂閫佹暟鎹鍦板潃
 * @retval      鍙戦佸畬鎴愮姸鎬
 *   @arg       0    : 鍙戦佹垚鍔
 *   @arg       1    : 杈惧埌鏈澶у彂閫佹鏁,澶辫触
 *   @arg       0XFF : 鍏朵粬閿欒
 */
uint8_t nrf24l01_tx_packet(uint8_t *ptxbuf)
{
    uint8_t sta;
    uint8_t rval = 0XFF;
    
    NRF24L01_CE(0);
    nrf24l01_write_buf(WR_TX_PLOAD, ptxbuf, TX_PLOAD_WIDTH);    /* 鍐欐暟鎹埌TX BUF  TX_PLOAD_WIDTH涓瓧鑺 */
    NRF24L01_CE(1);                     /* 鍚姩鍙戦 */

    while (NRF24L01_IRQ != 0);          /* 绛夊緟鍙戦佸畬鎴 */

    sta = nrf24l01_read_reg(STATUS);    /* 璇诲彇鐘舵佸瘎瀛樺櫒鐨勫 */
    nrf24l01_write_reg(NRF_WRITE_REG + STATUS, sta);    /* 娓呴櫎TX_DS鎴朚AX_RT涓柇鏍囧織 */

    if (sta & MAX_TX)   /* 杈惧埌鏈澶ч噸鍙戞鏁 */
    {
        nrf24l01_write_reg(FLUSH_TX, 0xff);             /* 娓呴櫎TX FIFO瀵勫瓨鍣 */
        rval = 1;
    }

    if (sta & TX_OK)    /* 鍙戦佸畬鎴 */
    {
        rval = 0;       /* 鏍囪鍙戦佹垚鍔 */
    }

    return rval;        /* 杩斿洖缁撴灉 */
}

/**
 * @brief       鍚姩NRF24L01鎺ユ敹涓娆℃暟鎹(鏁版嵁闀垮害 = RX_PLOAD_WIDTH)
 * @param       prxbuf : 鎺ユ敹鏁版嵁缂撳啿鍖洪鍦板潃
 * @retval      鎺ユ敹瀹屾垚鐘舵
 *   @arg       0 : 鎺ユ敹鎴愬姛
 *   @arg       1 : 澶辫触
 */
uint8_t nrf24l01_rx_packet(uint8_t *prxbuf)
{
    uint8_t sta;
    uint8_t rval = 1;
    
    sta = nrf24l01_read_reg(STATUS);                            /* 璇诲彇鐘舵佸瘎瀛樺櫒鐨勫 */
    nrf24l01_write_reg(NRF_WRITE_REG + STATUS, sta);            /* 娓呴櫎TX_DS鎴朚AX_RT涓柇鏍囧織 */

    if (sta & RX_OK)    /* 鎺ユ敹鍒版暟鎹 */
    {
        nrf24l01_read_buf(RD_RX_PLOAD, prxbuf, RX_PLOAD_WIDTH); /* 璇诲彇鏁版嵁 */
        nrf24l01_write_reg(FLUSH_RX, 0xff);                     /* 娓呴櫎RX FIFO瀵勫瓨鍣 */
        rval = 0;       /* 鏍囪鎺ユ敹瀹屾垚 */
    }

    return rval;        /* 杩斿洖缁撴灉 */
}

/**
 * @brief       NRF24L01杩涘叆鎺ユ敹妯″紡
 *   @note      璁剧疆RX鍦板潃,鍐橰X鏁版嵁瀹藉害,閫夋嫨RF棰戦亾,娉㈢壒鐜囧拰LNA HCURR
 *              褰揅E鍙橀珮鍚,鍗宠繘鍏X妯″紡,骞跺彲浠ユ帴鏀舵暟鎹簡
 * @param       鏃
 * @retval      鏃
 */
void nrf24l01_rx_mode(void)
{
    NRF24L01_CE(0);
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH);    /* 鍐橰X鑺傜偣鍦板潃 */

    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);        /* 浣胯兘閫氶亾0鐨勮嚜鍔ㄥ簲绛 */
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    /* 浣胯兘閫氶亾0鐨勬帴鏀跺湴鍧 */
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, 40);          /* 璁剧疆RF閫氫俊棰戠巼 */
    nrf24l01_write_reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);   /* 閫夋嫨閫氶亾0鐨勬湁鏁堟暟鎹搴 */
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     /* 璁剧疆TX鍙戝皠鍙傛暟,0db澧炵泭,2Mbps,浣庡櫔澹板鐩婂紑鍚 */
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0f);       /* 閰嶇疆鍩烘湰宸ヤ綔妯″紡鐨勫弬鏁;PWR_UP,EN_CRC,16BIT_CRC,鎺ユ敹妯″紡 */
    NRF24L01_CE(1); /* CE涓洪珮,杩涘叆鎺ユ敹妯″紡 */
}

/**
 * @brief       NRF24L01杩涘叆鍙戦佹ā寮
 *   @note      璁剧疆TX鍦板潃,鍐橳X鏁版嵁瀹藉害,璁剧疆RX鑷姩搴旂瓟鐨勫湴鍧,濉厖TX鍙戦佹暟鎹,閫夋嫨RF棰戦亾,娉㈢壒鐜囧拰
 *              LNA HCURR,PWR_UP,CRC浣胯兘
 *              褰揅E鍙橀珮鍚,鍗宠繘鍏X妯″紡,骞跺彲浠ュ彂閫佹暟鎹簡, CE涓洪珮澶т簬10us,鍒欏惎鍔ㄥ彂閫.
 * @param       鏃
 * @retval      鏃
 */
void nrf24l01_tx_mode(void)
{
    NRF24L01_CE(0);
    nrf24l01_write_buf(NRF_WRITE_REG + TX_ADDR, (uint8_t *)TX_ADDRESS, TX_ADR_WIDTH);       /* 鍐橳X鑺傜偣鍦板潃 */
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH);    /* 璁剧疆RX鑺傜偣鍦板潃,涓昏涓轰簡浣胯兘ACK */

    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);        /* 浣胯兘閫氶亾0鐨勮嚜鍔ㄥ簲绛 */
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    /* 浣胯兘閫氶亾0鐨勬帴鏀跺湴鍧 */
    nrf24l01_write_reg(NRF_WRITE_REG + SETUP_RETR, 0x1a);   /* 璁剧疆鑷姩閲嶅彂闂撮殧鏃堕棿:500us + 86us;鏈澶ц嚜鍔ㄩ噸鍙戞鏁:10娆 */
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, 40);          /* 璁剧疆RF閫氶亾涓40 */
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     /* 璁剧疆TX鍙戝皠鍙傛暟,0db澧炵泭,2Mbps,浣庡櫔澹板鐩婂紑鍚 */
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0e);       /* 閰嶇疆鍩烘湰宸ヤ綔妯″紡鐨勫弬鏁;PWR_UP,EN_CRC,16BIT_CRC,鎺ユ敹妯″紡,寮鍚墍鏈変腑鏂 */
    NRF24L01_CE(1); /* CE涓洪珮,10us鍚庡惎鍔ㄥ彂閫 */
}


/**
 * @brief  B妯″潡閰嶇疆1锛氬垏鎹负銆屾帴鏀禔鏁版嵁銆嶇殑妯″紡锛圓-B鍙傛暟锛
 * @note   鍩轰簬搴撳嚱鏁皀rf24l01_rx_mode()淇敼锛岄傞厤ADDR_AB銆丆H_AB
 */
void B_nrf24l01_switch_rx_from_A(void)
{
    NRF24L01_CE(0);  // 鎷変綆CE锛岃繘鍏ラ厤缃ā寮忥紙鍋滄褰撳墠妯″紡锛
    
    // 1. 鍐橰X_ADDR_P0锛圓鈫払鐨勫湴鍧 ADDR_AB锛
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_AB, ADDR_AB_LEN);
    
    // 2. 搴撳嚱鏁伴粯璁ら厤缃紙淇濇寔涓嶅彉锛屼粎淇敼棰戦亾鍜屽湴鍧锛
    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);        // 浣胯兘閫氶亾0鑷姩搴旂瓟锛堝悜A鍥炰紶ACK锛
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // 浣胯兘閫氶亾0鎺ユ敹鍦板潃
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, CH_AB);       // 鍒囨崲鍒癆-B棰戦亾 30
    nrf24l01_write_reg(NRF_WRITE_REG + RX_PW_P0, DATA_WIDTH); // 鏁版嵁闀垮害32瀛楄妭
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db澧炵泭锛2Mbps锛屼綆鍣０澧炵泭寮鍚
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0f);       // 鎺ユ敹妯″紡锛孭WR_UP锛孍N_CRC
    
    NRF24L01_CE(1);  // 鎷夐珮CE锛岃繘鍏ユ帴鏀剁姸鎬
//    HAL_Delay(10);   // 寤舵椂淇濊瘉閰嶇疆鐢熸晥锛岄伩鍏嶆ā寮忓啿绐
}

/**
 * @brief  B妯″潡閰嶇疆2锛氬垏鎹负銆屽彂閫佹暟鎹粰C銆嶇殑妯″紡锛圔-C鍙傛暟锛
 * @note   鍩轰簬搴撳嚱鏁皀rf24l01_tx_mode()淇敼锛岄傞厤ADDR_BC銆丆H_BC
 */
void B_nrf24l01_switch_tx_to_C(void)
{
    NRF24L01_CE(0);  // 鎷変綆CE锛岃繘鍏ラ厤缃ā寮忥紙鍋滄褰撳墠妯″紡锛
    
    // 1. 鍐橳X鍦板潃锛圔鈫扖鐨勫湴鍧 ADDR_BC锛
    nrf24l01_write_buf(NRF_WRITE_REG + TX_ADDR, ADDR_BC, ADDR_BC_LEN);
    // 2. 鍐橰X_ADDR_P0锛堜娇鑳紸CK锛屽繀椤讳笌C鐨凾X鍦板潃涓鑷达紝鍗矨DDR_BC锛
    nrf24l01_write_buf(NRF_WRITE_REG + RX_ADDR_P0, ADDR_BC, ADDR_BC_LEN);
    
    // 3. 搴撳嚱鏁伴粯璁ら厤缃紙淇濇寔涓嶅彉锛屼粎淇敼棰戦亾鍜屽湴鍧锛
    nrf24l01_write_reg(NRF_WRITE_REG + EN_AA, 0x01);        // 浣胯兘閫氶亾0鑷姩搴旂瓟
    nrf24l01_write_reg(NRF_WRITE_REG + EN_RXADDR, 0x01);    // 浣胯兘閫氶亾0鎺ユ敹鍦板潃
    nrf24l01_write_reg(NRF_WRITE_REG + SETUP_RETR, 0x1a);   // 鑷姩閲嶅彂锛10娆★紝闂撮殧500us+86us
    nrf24l01_write_reg(NRF_WRITE_REG + RF_CH, CH_BC);       // 鍒囨崲鍒癇-C棰戦亾 40
    nrf24l01_write_reg(NRF_WRITE_REG + RF_SETUP, 0x0f);     // 0db澧炵泭锛2Mbps锛屼綆鍣０澧炵泭寮鍚
    nrf24l01_write_reg(NRF_WRITE_REG + CONFIG, 0x0e);       // 鍙戦佹ā寮忥紝PWR_UP锛孍N_CRC
    
//    HAL_Delay(10);   // 寤舵椂淇濊瘉閰嶇疆鐢熸晥锛屾棤闇绔嬪嵆鎷夐珮CE锛堝彂閫佹椂鐢眓rf24l01_tx_packet瑙﹀彂锛
}
void NRF_check(void)
{
    uint32_t attempt;
    uint8_t online = 0;

    for (attempt = 0; attempt < NRF_CHECK_MAX_TRIES; attempt++)
    {
        if (nrf24l01_check() == 0)
        {
            online = 1;
            break;
        }
        delay_ms(NRF_CHECK_RETRY_DELAY_MS);
    }

    if (online)
    {
        printf("NRF24L01 online\r\n");
    }
    else
    {
        printf("NRF24L01 offline; continue without remote\r\n");
    }

    /* Configure the radio and let the scheduler/UART start even when the
     * hardware probe failed.  A later radio reconnect is handled by the
     * normal receive task without holding the whole controller in reset. */
	B_nrf24l01_switch_rx_from_A();
	LED1(1);
}






