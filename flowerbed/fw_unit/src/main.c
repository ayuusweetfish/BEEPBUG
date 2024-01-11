#include <stm32g0xx_hal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define ACT_LED_PIN   GPIO_PIN_15
#define ACT_LED_PORT  GPIOC

// #define RELEASE
#ifndef RELEASE
static uint8_t swv_buf[256];
static size_t swv_buf_ptr = 0;
__attribute__ ((noinline, used))
void swv_trap_line()
{
  *(volatile char *)swv_buf;
}
static inline void swv_putchar(uint8_t c)
{
  // ITM_SendChar(c);
  if (c == '\n') {
    swv_buf[swv_buf_ptr >= sizeof swv_buf ?
      (sizeof swv_buf - 1) : swv_buf_ptr] = '\0';
    swv_trap_line();
    swv_buf_ptr = 0;
  } else if (++swv_buf_ptr <= sizeof swv_buf) {
    swv_buf[swv_buf_ptr - 1] = c;
  }
}
static void swv_printf(const char *restrict fmt, ...)
{
  char s[256];
  va_list args;
  va_start(args, fmt);
  int r = vsnprintf(s, sizeof s, fmt, args);
  for (int i = 0; i < r && i < sizeof s - 1; i++) swv_putchar(s[i]);
  if (r >= sizeof s) {
    for (int i = 0; i < 3; i++) swv_putchar('.');
    swv_putchar('\n');
  }
}
#else
#define swv_printf(...)
#endif

TIM_HandleTypeDef tim14 = { 0 };
I2C_HandleTypeDef i2c2 = { 0 };

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init;

  // Act LED
  gpio_init = (GPIO_InitTypeDef){
    .Pin = ACT_LED_PIN,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(ACT_LED_PORT, &gpio_init);
  HAL_GPIO_WritePin(ACT_LED_PORT, ACT_LED_PIN, 0);

  // SWD (PA13, PA14)
  gpio_init.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio_init.Mode = GPIO_MODE_AF_PP; // Pass over control to AF peripheral
  gpio_init.Alternate = GPIO_AF0_SWJ;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  // ======== Clocks ========
  RCC_OscInitTypeDef osc_init = { 0 };
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc_init.HSEState = RCC_HSE_BYPASS;
  osc_init.PLL.PLLState = RCC_PLL_ON;
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc_init.PLL.PLLM = RCC_PLLM_DIV1;  // VCO input 12.288 MHz (2.66 ~ 16 MHz)
  osc_init.PLL.PLLN = 12;             // VCO output 147.456 MHz (64 ~ 344 MHz)
  osc_init.PLL.PLLP = RCC_PLLP_DIV4;  // PLLPCLK 36.864 MHz
  osc_init.PLL.PLLR = RCC_PLLR_DIV4;  // PLLRCLK 36.864 MHz
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // PLLRCLK
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  for (int i = 0; i < 2; i++) {
    HAL_GPIO_WritePin(ACT_LED_PORT, ACT_LED_PIN, 1); HAL_Delay(100);
    HAL_GPIO_WritePin(ACT_LED_PORT, ACT_LED_PIN, 0); HAL_Delay(100);
  }

  // ======== Timer ========
  __HAL_RCC_TIM14_CLK_ENABLE();
  tim14 = (TIM_HandleTypeDef){
    .Instance = TIM14,
    .Init = {
      // 36.864 MHz
      .Prescaler = 2304 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 16000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_Base_Init(&tim14);
  HAL_TIM_Base_Start_IT(&tim14);
  __HAL_TIM_ENABLE_IT(&tim14, TIM_IT_UPDATE);
  HAL_NVIC_SetPriority(TIM14_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM14_IRQn);

  // ======== I2C ========
  gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF6_I2C2;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  __HAL_RCC_I2C2_CLK_ENABLE();
  i2c2 = (I2C_HandleTypeDef){
    .Instance = I2C2,
    .Init = {
      // RM0454 Rev 5, pp. 711, 726, 738 (examples), 766
      // APB = 36.864 MHz, f_SCL = __ kHz
      // PRESC = 15, SCLDEL = 0x4, SDADEL = 0x2,
      // SCLH = 0x0F, SCLH = 0x0F, SCLL = 0x13
      .Timing = 0xF0420F13,
      .OwnAddress1 = 0x00,
      .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    },
  };
  HAL_I2C_Init(&i2c2);

  // Verify reference registers for VL53L0X
  uint8_t ref_reg[3] = {0x01, 0x02, 0x03};
  HAL_StatusTypeDef result_ref =
    HAL_I2C_Mem_Read(&i2c2, 0x29 << 1, 0xC0, I2C_MEMADD_SIZE_8BIT, ref_reg, 3, 1000);
  if (result_ref != HAL_OK ||
      ref_reg[0] != 0xEE || ref_reg[1] != 0xAA || ref_reg[2] != 0x10) {
    swv_printf("%d %d %02x %02x %02x\n", (int)result_ref, (int)i2c2.ErrorCode,
      (unsigned)ref_reg[0], (unsigned)ref_reg[1], (unsigned)ref_reg[2]);
    while (1) {
      HAL_GPIO_WritePin(ACT_LED_PORT, ACT_LED_PIN, 1); HAL_Delay(100);
      HAL_GPIO_WritePin(ACT_LED_PORT, ACT_LED_PIN, 0); HAL_Delay(100);
    }
  }

  while (1) { }
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void TIM14_IRQHandler()
{
  HAL_TIM_IRQHandler(&tim14);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *tim14)
{
  static uint32_t count = 0, parity = 0;
  swv_printf("timer\n");
  HAL_GPIO_WritePin(ACT_LED_PORT, ACT_LED_PIN, parity ^= 1);
}
