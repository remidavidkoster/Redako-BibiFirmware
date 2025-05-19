#include "main.h"
#include "config.h"
#include "eth/artnet.h"
#include "eth/dhcp_server.h"
#include "eth/http_custom.h"
#include "eth/ncm_netif.h"
#include "lwip/apps/httpd.h"
#include "lwip/apps/mdns.h"
#include "lwip/igmp.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "ncm_device.h"
#include "oled/oled.h"
#include "profiling.h"
#include "systimer.h"
#include "usart.h"
#include "usb.h"

static void Clock_Init(void);
static void GPIO_Init(void);
static void ReadPortConfig(void);

static unsigned char portConfig[] = {0, 0, 0, 0};
static struct netif ncm_if;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    // Jump to bootloader triggered
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    if (TAMP->BKP0R == 0xF0) {
        PWR->CR1 |= PWR_CR1_DBP;
        TAMP->BKP0R = 0x00;
        PWR->CR1 &= ~PWR_CR1_DBP;
        SYSCFG->MEMRMP |= 0x01;
        void (*SysMemBootJump)(void) = (void (*)(void))(*((unsigned long *)(STM32_SYSMEM + 4)));
        __set_MSP(*(unsigned long *)STM32_SYSMEM);
        SysMemBootJump();
    }

    Clock_Init();
    Systick_Init();
    GPIO_Init();

    USB_Init();

    lwip_init();
    httpd_init();
    igmp_init();
    mdns_resp_init();

    netif_add(&ncm_if, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY, NULL, ncm_netif_init, netif_input);
    netif_set_default(&ncm_if);
    netif_set_up(&ncm_if);

    ReadPortConfig();

    DhcpServer_Init();
    Config_Init(&ncm_if, portConfig);
    USART_Init(Config_GetActive());
    ArtNet_Init(&ncm_if);
    httpc_init(&ncm_if);

    igmp_start(&ncm_if);
    mdns_resp_add_netif(&ncm_if, "artnet", 3600);

    OLED_Init();

    unsigned int last_inputTick = 0;
    unsigned int last_forcedInputTick = 0;
    unsigned int resetTimer = 0;

    while (1) {
        ncm_netif_poll(&ncm_if);

        if (sys_now() - last_inputTick > 24) {
            if (sys_now() - last_forcedInputTick > 1000) {
                ArtNet_InputTick(1);
                last_forcedInputTick = sys_now();
            } else {
                ArtNet_InputTick(0);
            }

            NCM_FlushTx();
            last_inputTick = sys_now();
        }

        sys_check_timeouts();
        ArtNet_TimeoutTick();
        httpc_timeout();
        OLED_Tick();

        // Reset button
        if (GPIOA->IDR & (1 << 5)) {
            if (resetTimer != 0) {
                if (sys_now() - resetTimer > 5000) {
                    Config_Reset();
                    resetTimer = 0;
                }
            } else {
                resetTimer = sys_now();
            }
        } else {
            resetTimer = 0;
        }

        USART_Tick();
    }
}

/**
 * @brief Get the configuration for each port based on the dip switches
 * @details
 * GND = Output
 * 3.3V = Input
 */
static void ReadPortConfig() {
    // Enable inputs on PortA0..3 with pull down
    GPIOA->MODER &= ~(GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk | GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD1_1 | GPIO_PUPDR_PUPD2_1 | GPIO_PUPDR_PUPD3_1;

    // detect configuration, port mapping is reverse to ports
    for (int i = 0; i < 4; i++) {
        if ((GPIOA->IDR & (1 << (3 - i))) != 0) {
            portConfig[i] = ARTNET_INPUT;
        } else {
            portConfig[i] = ARTNET_OUTPUT;
        }
    }

    // Disable Ports
    GPIOA->MODER |= (GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk | GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
static void Clock_Init(void) {
	// Enable HSE and HSI clock sources
	RCC->CR |= RCC_CR_HSEON | RCC_CR_HSION;

	// Disable PLL and wait until it's off
	RCC->CR &= ~RCC_CR_PLLON;
	while (RCC->CR & RCC_CR_PLLRDY) { }

	// Configure PLL for 144MHz output from 8MHz crystal
	// 8MHz / 1 (PLLM=1) = 8MHz PLL input
	// 8MHz * 36 (PLLN=36) = 288MHz VCO
	// 288MHz / 2 (PLLR=2) = 144MHz system clock
	// 288MHz / 6 (PLLQ=6) = 48MHz for USB/ADC
	RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | // HSE as PLL source
	                RCC_PLLCFGR_PLLM_0 |     // PLLM = 1 (bits: 001)
	                (36 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 36
	                RCC_PLLCFGR_PLLR_0 |     // PLLR = 2 (bits: 01)
	                RCC_PLLCFGR_PLLQ_1;      // PLLQ = 6 (bits: 10)

	// Enable PLLR and PLLQ outputs
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLQEN;

	// Enable PLL
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY)) { } // Wait until PLL is ready

	// Select PLL as main clock, temporarily set AHB/2 for transition
	RCC->CFGR |= RCC_CFGR_HPRE_3 | RCC_CFGR_SW_PLL;

	// Exit R1 power mode for full speed operation
	PWR->CR5 &= ~PWR_CR5_R1MODE;

	// Update flash latency for 144MHz operation (5 wait states for voltage range 1)
	unsigned int latency = FLASH->ACR;
	latency &= ~0xFF;
	latency |= 5; // 5 wait states for 144MHz in range 1
	FLASH->ACR = latency;
	while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != 5) { }

	// Set AHB prescaler back to /1 for full speed
	RCC->CFGR &= ~RCC_CFGR_HPRE_3;

	// Select & Enable IO Clocks
	// Set USB clock source to PLL-Q
	RCC->CCIPR = RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_ADC12SEL_1;
	RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN | RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN | RCC_APB1ENR1_UART4EN | RCC_APB1ENR1_USART3EN | RCC_APB1ENR1_USART2EN | RCC_APB1ENR1_TIM2EN;
	RCC->APB1ENR2 |= RCC_APB1ENR2_I2C4EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// Enable DMAMUX & DMA Clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;

	// Configure RTC-Clock for Backup registers, if necessary
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0) {
	    PWR->CR1 |= PWR_CR1_DBP;
	    RCC->BDCR |= 0x02 << RCC_BDCR_RTCSEL_Pos;
	    RCC->BDCR |= RCC_BDCR_RTCEN;
	    PWR->CR1 &= ~PWR_CR1_DBP;
	}

	// Wait until PLL is used as system clock
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) { }

//	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//	  /** Configure the main internal regulator output voltage
//	  */
//	  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//	  /** Initializes the RCC Oscillators according to the specified parameters
//	  * in the RCC_OscInitTypeDef structure.
//	  */
//	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
//	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//	  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
//	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//	  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
//	  RCC_OscInitStruct.PLL.PLLN = 18;
//	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
//	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
//	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//	  {
////	    Error_Handler();
//	  }
//
//	  /** Initializes the CPU, AHB and APB buses clocks
//	  */
//	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//	  {
////	    Error_Handler();
//	  }


    SystemCoreClockUpdate();
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void GPIO_Init(void) {
    /*  PA
        5 > Config Reset
        11 & 12 > USB
        13 & 14 > SWDIO & SWCLK
        15 > DMX 1 Direction Output
     */
    GPIOA->MODER &= ~(GPIO_MODER_MODE15 | GPIO_MODER_MODE5);
    GPIOA->MODER |= GPIO_MODER_MODE15_0;
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD15);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD5_1;

    /* PB
      1, 2, 13, 14 > OLED Navigation
      15 > OLED Sense
      3 & 4 > DMX 4 TX,RX
      5,6,7 > DMX 3 DE,TX,RX
      8 > BOOT0
      10,11,12 > DMX 2 TX,RX,DE
     */
    GPIOB->AFR[0] = (7 << GPIO_AFRL_AFSEL3_Pos) | (7 << GPIO_AFRL_AFSEL4_Pos) | (7 << GPIO_AFRL_AFSEL6_Pos) | (7 << GPIO_AFRL_AFSEL7_Pos);
    GPIOB->AFR[1] = (7 << GPIO_AFRH_AFSEL10_Pos) | (7 << GPIO_AFRH_AFSEL11_Pos);
    GPIOB->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7 | GPIO_MODER_MODE8 | GPIO_MODER_MODE10 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12 | GPIO_MODER_MODE13 | GPIO_MODER_MODE14 | GPIO_MODER_MODE15);
    GPIOB->MODER |= GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1 | GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1 | GPIO_MODER_MODE12_0;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD1_1 | GPIO_PUPDR_PUPD2_1 | GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD7_0 | GPIO_PUPDR_PUPD11_0 | GPIO_PUPDR_PUPD13_1 | GPIO_PUPDR_PUPD14_1 | GPIO_PUPDR_PUPD15_1;

    /* PC
      6, 7 > I²C SCL, SDA OLED
      10,11 > DMX 1 TX,RX
      12  > DMX 4 DE
     */
    GPIOC->AFR[0] = (8 << GPIO_AFRL_AFSEL6_Pos) | (8 << GPIO_AFRL_AFSEL7_Pos);
    GPIOC->AFR[1] = (5 << GPIO_AFRH_AFSEL10_Pos) | (5 << GPIO_AFRH_AFSEL11_Pos);
    GPIOC->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODER7 | GPIO_MODER_MODE10 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12);
    GPIOC->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1 | GPIO_MODER_MODE12_0;
    GPIOC->PUPDR |= GPIO_PUPDR_PUPD11_0;
}

void NMI_Handler() {
}

void HardFault_Handler() {
    while (1) {
    }
}
