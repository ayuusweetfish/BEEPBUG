#include <stm32g0xx_hal.h>
#include "vl53l0x/vl53l0x_api.h"

#define uQOA_IMPL
#include "uqoa.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define ACT_LED_PORT  GPIOC
#define ACT_LED_PIN   GPIO_PIN_15

#define W25Q_CS_PORT  GPIOA
#define W25Q_CS_PIN   GPIO_PIN_2

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
I2S_HandleTypeDef i2s1 = { 0 };
SPI_HandleTypeDef spi2 = { 0 };

#pragma GCC optimize("O3")
static inline void spi2_transmit(uint8_t *data, size_t size)
{
  HAL_SPI_Transmit(&spi2, data, size, 1000); return;
/*
  for (int i = 0; i < size; i++) {
    while (!(SPI2->SR & SPI_SR_TXE)) { }
    SPI2->DR = data[i];
  }
  while (!(SPI2->SR & SPI_SR_TXE)) { }
  while ((SPI2->SR & SPI_SR_BSY)) { }
  // Clear OVR flag
  (void)SPI2->DR;
  (void)SPI2->SR;
*/
}

#pragma GCC optimize("O3")
static inline void spi2_receive(uint8_t *data, size_t size)
{
  HAL_SPI_Receive(&spi2, data, size, 1000); return;
/*
  for (int i = 0; i < size; i++) {
    while (!(SPI2->SR & SPI_SR_TXE)) { }
    SPI2->DR = 0;
    while (!(SPI2->SR & SPI_SR_RXNE)) { }
    data[i] = (uint8_t)SPI2->DR;
  }
  while (!(SPI2->SR & SPI_SR_TXE)) { }
  while ((SPI2->SR & SPI_SR_BSY)) { }
*/
}

#pragma GCC optimize("O3")
static inline void spi_flash_tx_rx(
  GPIO_TypeDef *cs_port, uint32_t cs_pin,
  uint8_t *txbuf, size_t txsize,
  uint8_t *rxbuf, size_t rxsize
) {
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 16;
  spi2_transmit(txbuf, txsize);
  if (rxsize != 0) {
    while (SPI1->SR & SPI_SR_BSY) { }
    spi2_receive(rxbuf, rxsize);
  }
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 0;
}

#define flash_cmd(_cmd) \
  spi_flash_tx_rx(W25Q_CS_PORT, W25Q_CS_PIN, (_cmd), sizeof (_cmd), NULL, 0)
#define flash_cmd_sized(_cmd, _cmdlen) \
  spi_flash_tx_rx(W25Q_CS_PORT, W25Q_CS_PIN, (_cmd), (_cmdlen), NULL, 0)
#define flash_cmd_bi(_cmd, _rxbuf) \
  spi_flash_tx_rx(W25Q_CS_PORT, W25Q_CS_PIN, (_cmd), sizeof (_cmd), (_rxbuf), sizeof (_rxbuf))
#define flash_cmd_bi_sized(_cmd, _cmdlen, _rxbuf, _rxlen) \
  spi_flash_tx_rx(W25Q_CS_PORT, W25Q_CS_PIN, (_cmd), (_cmdlen), (_rxbuf), (_rxlen))

uint8_t flash_status0()
{
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 16;
  uint8_t op_read_status[] = {0x05};
  spi2_transmit(op_read_status, sizeof op_read_status);
  uint8_t status0;
  spi2_receive(&status0, 1);
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 0;
  return status0;
}

__attribute__ ((noinline))
uint32_t flash_status_all()
{
  uint8_t op_read_status;
  uint8_t status[3];
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 16;
  op_read_status = 0x05; spi2_transmit(&op_read_status, 1); spi2_receive(&status[0], 1);
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 0;
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 16;
  op_read_status = 0x35; spi2_transmit(&op_read_status, 1); spi2_receive(&status[1], 1);
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 0;
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 16;
  op_read_status = 0x15; spi2_transmit(&op_read_status, 1); spi2_receive(&status[2], 1);
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 0;
  return ((uint32_t)status[2] << 16) | ((uint32_t)status[1] << 8) | status[0];
}

void flash_wait_poll(uint32_t interval_us)
{
  uint8_t status0;
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 16;
  uint8_t op_read_status[] = {0x05};
  spi2_transmit(op_read_status, sizeof op_read_status);
  do {
    // dwt_delay(interval_us * CYC_MICROSECOND);
    spi2_receive(&status0, 1);
    // swv_printf("BUSY = %u, SysTick = %lu\n", status0 & 1, HAL_GetTick());
  } while (status0 & 1);
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 0;
}

void flash_id(uint8_t jedec[3], uint8_t uid[4])
{
  uint8_t op_read_jedec[] = {0x9F};
  flash_cmd_bi_sized(op_read_jedec, sizeof op_read_jedec, jedec, 3);
  uint8_t op_read_uid[] = {0x4B, 0x00, 0x00, 0x00, 0x00};
  flash_cmd_bi_sized(op_read_uid, sizeof op_read_uid, uid, 4);
}

void flash_erase_4k(uint32_t addr)
{
  addr &= ~0xFFF;
  uint8_t op_write_enable[] = {0x06};
  flash_cmd(op_write_enable);
  uint8_t op_sector_erase[] = {
    0x20, // Sector Erase
    (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, (addr >> 0) & 0xFF,
  };
  flash_cmd(op_sector_erase);
  // Wait for completion (t_SE max. = 400 ms, typ. = 40 ms)
  flash_wait_poll(1000);
}

void flash_erase_64k(uint32_t addr)
{
  addr &= ~0xFFFF;
  uint8_t op_write_enable[] = {0x06};
  flash_cmd(op_write_enable);
  uint8_t op_sector_erase[] = {
    0xD8, // 64KB Block Erase
    (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, (addr >> 0) & 0xFF,
  };
  flash_cmd(op_sector_erase);
  // Wait for completion (t_BE2 max. = 2000 ms, typ. = 150 ms)
  flash_wait_poll(1000);
}

void flash_erase_chip()
{
  uint8_t op_write_enable[] = {0xC7};
  flash_cmd(op_write_enable);
  uint8_t op_chip_erase[] = {
    0x20, // Chip Erase
  };
  flash_cmd(op_chip_erase);
  // Wait for completion (t_CE max. = 25000 ms, typ. = 5000 ms)
  flash_wait_poll(10000);
}

void flash_write_page(uint32_t addr, uint8_t *data, size_t size)
{
  // assert(size > 0 && size <= 256);
  uint8_t op_write_enable[] = {0x06};
  flash_cmd(op_write_enable);
  uint8_t op_page_program[260] = {
    0x02, // Page Program
    (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, (addr >> 0) & 0xFF,
  };
  for (size_t i = 0; i < size; i++)
    op_page_program[4 + i] = data[i];
  flash_cmd_sized(op_page_program, 4 + size);
  // Wait for completion (t_PP max. = 3 ms, typ. = 0.4 ms)
  flash_wait_poll(100);
}

__attribute__ ((noinline))
void flash_read(uint32_t addr, uint8_t *data, size_t size)
{
  uint8_t op_read_data[] = {
    0x03, // Read Data
    (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, (addr >> 0) & 0xFF,
  };
  flash_cmd_bi_sized(op_read_data, sizeof op_read_data, data, size);
}

void flash_read_dma(uint32_t addr, uint8_t *data, size_t size)
{
  uint8_t op_read_data[] = {
    0x03, // Read Data
    (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, (addr >> 0) & 0xFF,
  };
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 16;
  spi2_transmit(op_read_data, sizeof op_read_data);
  HAL_SPI_TransmitReceive_DMA(&spi2, data, data, size);
}

void flash_read_dma_wait_end()
{
  while (spi2.Instance->SR & SPI_SR_BSY) { }
  W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 0;
}

uint8_t flash_test_write_buf[256 * 8];

__attribute__ ((noinline))
void flash_test_write(uint32_t addr, size_t size)
{
  for (uint32_t block_start = 0; block_start < size; block_start += 256) {
    uint32_t block_size = 256;
    if (block_start + block_size >= size)
      block_size = size - block_start;
    flash_write_page(
      addr + block_start,
      flash_test_write_buf + block_start,
      block_size
    );
  }
}

void flash_test_write_breakpoint()
{
  swv_printf("all status = %06x\n", flash_status_all());
  if (*(volatile uint32_t *)flash_test_write_buf == 0x11223344) {
    uint8_t data[4] = {1, 2, 3, 4};
    flash_read(0, data, sizeof data);
    for (int i = 0; i < 4; i++) swv_printf("%d\n", data[i]);
    flash_erase_4k(0);
    flash_erase_64k(0);
    flash_erase_chip();
    flash_test_write(0, 1);
  }
  while (1) { }
}

// Audio data address range
static uint32_t audio_compressed_addr_start, audio_compressed_addr_end;
// Buffer for raw data
#define N_AUDIO_COMPR_BUF 128
static uint64_t audio_compressed_buf[N_AUDIO_COMPR_BUF];
// Next address for reading
static uint32_t audio_compressed_addr;
// Whether the read is in progress
static volatile bool audio_read_in_progress = 0;
// Which half of the raw data the next read should target
static uint8_t audio_compressed_half;
// Raw data consumption pointer (into the entire buffer, without knowledge of the halves)
static size_t audio_compressed_ptr;

typedef struct __attribute__ ((packed, aligned(4))) { int16_t l, r; } stereo_sample_t;
static inline stereo_sample_t sample(int16_t x) { return (stereo_sample_t){x, x}; }
// Buffer for decoded samples
#define N_AUDIO_PCM_BUF 480
static stereo_sample_t audio_pcm_buf[N_AUDIO_PCM_BUF];
// Decoder state
static qoa_lms audio_dec_state;
// Block counter, resets at 256
static uint16_t audio_block_ctr;

// Volume (12-bit fixed point, 4096 corresponds to unity gain)
static volatile uint16_t audio_volume;

// 0 - normal running
// 1 - `audio_playback_init()` would like the running thread to hold
// 2 - the running thread (through `audio_decode()` has stopped all processing)
static volatile uint8_t audio_hold_state;

static void audio_read_start();
static void audio_decode(uint8_t into_half, bool ignore_hold_state);
static void audio_playback_init(uint32_t addr_start, uint32_t addr_end)
{
  // Raise a flag for 
  audio_hold_state = 1;
  while (audio_hold_state != 2) { }

  audio_compressed_addr_start = addr_start;
  audio_compressed_addr_end = addr_end;
  audio_compressed_addr = audio_compressed_addr_start;
  audio_compressed_half = 0;
  audio_compressed_ptr = 0;
  audio_block_ctr = 256;

  // Read the first samples and decode
  while (audio_read_in_progress) { }
  audio_read_start();
  while (audio_read_in_progress) { }
  audio_read_start();
  while (audio_read_in_progress) { }
  audio_decode(0, true);
  audio_decode(1, true);

  audio_hold_state = 0;
}

static void audio_read_start()
{
  audio_read_in_progress = true;
  if (audio_compressed_addr + (N_AUDIO_COMPR_BUF / 2) * 8 >= audio_compressed_addr_end) {
    for (int i = 0; i < N_AUDIO_PCM_BUF; i++) audio_pcm_buf[i] = sample(0);
    while (1) {
      HAL_GPIO_WritePin(ACT_LED_PORT, ACT_LED_PIN, 1); HAL_Delay(200);
      HAL_GPIO_WritePin(ACT_LED_PORT, ACT_LED_PIN, 0); HAL_Delay(100);
    }
  }
  uint64_t *data = audio_compressed_buf +
    (audio_compressed_half ? N_AUDIO_COMPR_BUF / 2 : 0);
  flash_read_dma(audio_compressed_addr,
    (uint8_t *)data, N_AUDIO_COMPR_BUF / 2 * 8);
  audio_compressed_addr += (N_AUDIO_COMPR_BUF / 2) * 8;
}
static void audio_read_mark_end()
{
  uint64_t *data = audio_compressed_buf +
    (audio_compressed_half ? N_AUDIO_COMPR_BUF / 2 : 0);
  for (int i = 0; i < N_AUDIO_COMPR_BUF / 2; i++)
    data[i] = __builtin_bswap64(data[i]);
/*
  // 64-bit printing might not be supported
  swv_printf("data offset %08x -> half %d:\n",
    audio_compressed_addr - (N_AUDIO_COMPR_BUF / 2) * 8, (int)audio_compressed_half);
  for (int i = 0; i < 16; i++)
    swv_printf("%08x%08x%c",
      (uint32_t)(data[i] >> 32),
      (uint32_t)(data[i] >> 0),
      i % 4 == 3 ? '\n' : ' ');
*/
  audio_compressed_half ^= 1;
  audio_read_in_progress = false;
}
static void audio_decode(uint8_t into_half, bool ignore_hold_state)
{
  if (audio_read_in_progress) {
    swv_printf("! decode requested while data read in progress\n");
    return;
  }
  if (!ignore_hold_state && (audio_hold_state == 1 || audio_hold_state == 2)) {
    audio_hold_state = 2;
    return;
  }

  const size_t buffer_half_sz = N_AUDIO_PCM_BUF / 2;
  const size_t n_blocks = buffer_half_sz / 20;

  uint64_t *data = audio_compressed_buf + audio_compressed_ptr;
  stereo_sample_t *pcm = audio_pcm_buf + (into_half ? buffer_half_sz : 0);
  uint64_t *data_half_end =
    audio_compressed_buf + (audio_compressed_half + 1) * (N_AUDIO_COMPR_BUF / 2);
  uint16_t vol = audio_volume;
  bool start_new_read = false;

  for (int i = 0; i < n_blocks; i++) {
    if (audio_block_ctr == 256) {
      // Load decoder state
      for (int i = 0; i < 4; i++)
        audio_dec_state.history[i] = (int16_t)(data[0] >> (16 * (3 - i)));
      for (int i = 0; i < 4; i++)
        audio_dec_state.weights[i] = (int16_t)(data[1] >> (16 * (3 - i)));
    /*
      swv_printf("Reload (at offset %08x): %08x%08x %08x%08x\n",
        data - audio_compressed_buf,
        (uint32_t)(data[0] >> 32), (uint32_t)(data[0] >> 0),
        (uint32_t)(data[1] >> 32), (uint32_t)(data[1] >> 0));
    */
      data += 2;
      if (data == data_half_end) {
        start_new_read = true;
        if (audio_compressed_half == 1) data = audio_compressed_buf;
        /* swv_printf("Past half (%d): first value is %08x%08x\n",
          audio_compressed_half,
          (uint32_t)(data[0] >> 32), (uint32_t)(data[0] >> 0)); */
      }
      audio_block_ctr = 0;
    }
    int16_t decoded_pcm[20];
    qoa_decode_slice(&audio_dec_state, *data, decoded_pcm);
    for (int i = 0; i < 20; i++)
      pcm[i] = sample(((int32_t)decoded_pcm[i] * vol) >> 12);
    /* if (n_called == 6 || start_new_read) {
      swv_printf("data: %08x%08x\ndecoded: ",
        (uint32_t)(data[0] >> 32), (uint32_t)(data[0] >> 0));
      for (int i = 0; i < 20; i++)
        swv_printf("%04x%c", (uint32_t)(uint16_t)pcm[i], i == 19 ? '\n' : ' ');
    } */
    data += 1;
    if (data == data_half_end) {
      start_new_read = true;
      if (audio_compressed_half == 1) data = audio_compressed_buf;
      /* swv_printf("Past half (%d): first value is %08x%08x\n",
        audio_compressed_half,
        (uint32_t)(data[0] >> 32), (uint32_t)(data[0] >> 0)); */
    }
    pcm += 20;
    audio_block_ctr += 1;
  }

  // Move data consumption pointer
  audio_compressed_ptr = data - audio_compressed_buf;
  if (start_new_read) {
    // swv_printf("raw data refill initiated\n");
    audio_read_start();
  }
}

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
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
      // RM0454 Rev 5, pp. 711, 726, 738 (examples), 766 (reg. map)
      // APB = 36.864 MHz
      // f_SCL = 368.64 kHz = APB / 100
      // PRESC = 9 (scaled clock APB / 10 = 0.2713 µs)
      // SCLDEL = 0x1, SDADEL = 0x1,
      // SCLH = 0x03 (0.8138 µs > 0.6 µs), SCLL = 0x05 (1.356 µs > 1.3 µs)
      .Timing = 0x90110305,
      .OwnAddress1 = 0x00,
      .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    },
  };
  HAL_I2C_Init(&i2c2);

  // ======== DMA for I2S ========
  __HAL_RCC_DMA1_CLK_ENABLE();
  // RM0454, Ch. 9.3.2 —
  // DMA channels are multiplexed instead of hard-wired like the 'F103C6
  DMA_HandleTypeDef dma_i2s1_tx;
  dma_i2s1_tx.Instance = DMA1_Channel1;
  dma_i2s1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
  dma_i2s1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  dma_i2s1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  dma_i2s1_tx.Init.MemInc = DMA_MINC_ENABLE;
  dma_i2s1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  dma_i2s1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  dma_i2s1_tx.Init.Mode = DMA_CIRCULAR;
  dma_i2s1_tx.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&dma_i2s1_tx);

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 15, 1);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(SPI1_IRQn, 15, 2);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);

  // ======== I2S ========
  gpio_init.Pin = GPIO_PIN_5 | GPIO_PIN_7;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF0_SPI1;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  gpio_init.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  __HAL_RCC_SPI1_CLK_ENABLE();
  i2s1 = (I2S_HandleTypeDef){
    .Instance = SPI1,
    .Init = {
      .Mode = I2S_MODE_MASTER_TX,
      .Standard = I2S_STANDARD_PHILIPS,
      .DataFormat = I2S_DATAFORMAT_16B,
      .MCLKOutput = I2S_MCLKOUTPUT_DISABLE,
      .AudioFreq = I2S_AUDIOFREQ_48K,
      .CPOL = I2S_CPOL_LOW,
    },
  };
  __HAL_LINKDMA(&i2s1, hdmatx, dma_i2s1_tx);
  HAL_I2S_Init(&i2s1);

  swv_printf("sys clock = %u\n", HAL_RCC_GetSysClockFreq());
  swv_printf("I2S clock = %u\n", HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2S1));
  // 36864000, divider is 36864000 / 16 / 48k = 48

  // ======== DMA for SPI ========
  DMA_HandleTypeDef dma_spi2_tx;
  dma_spi2_tx.Instance = DMA1_Channel2;
  dma_spi2_tx.Init.Request = DMA_REQUEST_SPI2_TX;
  dma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  dma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  dma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
  dma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  dma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  dma_spi2_tx.Init.Mode = DMA_NORMAL;
  dma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&dma_spi2_tx);

  DMA_HandleTypeDef dma_spi2_rx;
  dma_spi2_rx.Instance = DMA1_Channel3;
  dma_spi2_rx.Init.Request = DMA_REQUEST_SPI2_RX;
  dma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  dma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  dma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
  dma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  dma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  dma_spi2_rx.Init.Mode = DMA_NORMAL;
  dma_spi2_rx.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&dma_spi2_rx);

  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 15, 3);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  HAL_NVIC_SetPriority(SPI2_IRQn, 15, 4);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);

  // ======== SPI ========
  gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_3;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Alternate = GPIO_AF0_SPI2;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  gpio_init.Pin = GPIO_PIN_4;
  gpio_init.Alternate = GPIO_AF1_SPI2;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  gpio_init.Pin = W25Q_CS_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(W25Q_CS_PORT, &gpio_init);
  HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, 1);

  __HAL_RCC_SPI2_CLK_ENABLE();
  spi2.Instance = SPI2;
  spi2.Init.Mode = SPI_MODE_MASTER;
  spi2.Init.Direction = SPI_DIRECTION_2LINES;
  spi2.Init.CLKPolarity = SPI_POLARITY_LOW; // CPOL = 0
  spi2.Init.CLKPhase = SPI_PHASE_1EDGE;     // CPHA = 0
  spi2.Init.NSS = SPI_NSS_SOFT;
  spi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi2.Init.TIMode = SPI_TIMODE_DISABLE;
  spi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi2.Init.DataSize = SPI_DATASIZE_8BIT;
  spi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;  // APB / 2 = 18.432 MHz
  __HAL_LINKDMA(&spi2, hdmatx, dma_spi2_tx);
  __HAL_LINKDMA(&spi2, hdmarx, dma_spi2_rx);
  HAL_SPI_Init(&spi2);
  __HAL_SPI_ENABLE(&spi2);

  __HAL_DMA_ENABLE_IT(&dma_spi2_tx, (DMA_IT_TC | DMA_IT_TE));
  __HAL_DMA_ENABLE(&dma_spi2_tx);
  __HAL_DMA_ENABLE_IT(&dma_spi2_rx, (DMA_IT_TC | DMA_IT_TE));
  __HAL_DMA_ENABLE(&dma_spi2_rx);

  uint8_t jedec[3], uid[4];
  flash_id(jedec, uid);
  // Manufacturer = 0xef (Winbond)
  // Memory type = 0x40
  // Capacity = 0x15 (2^21 B = 2 MiB = 16 Mib)
  swv_printf("MF = %02x\nID = %02x %02x\nUID = %02x%02x%02x%02x\n",
    jedec[0], jedec[1], jedec[2], uid[0], uid[1], uid[2], uid[3]);
  // flash_test_write_breakpoint();

  audio_hold_state = 0;
  for (int i = 0; i < N_AUDIO_PCM_BUF; i++) audio_pcm_buf[i] = sample(0);
  HAL_I2S_Transmit_DMA(&i2s1, (uint16_t *)audio_pcm_buf, N_AUDIO_PCM_BUF * 2);

  audio_volume = 1024;
  audio_playback_init(0, 193504);

/*
  while (1) {
    HAL_Delay(3000);
    audio_playback_init(0, 193504);
  }
*/

  VL53L0X_Dev_t dev;
  VL53L0X_Error err;
  VL53L0X_ResetDevice(&dev);

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

  err = VL53L0X_DataInit(&dev);
  if (err != VL53L0X_ERROR_NONE) swv_printf("err1 = %d\n", (int)err);
  uint16_t osc_cal = 0x97AA;
  err = VL53L0X_RdWord(&dev, VL53L0X_REG_OSC_CALIBRATE_VAL, &osc_cal);
  if (err != VL53L0X_ERROR_NONE) swv_printf("err2 = %d\n", (int)err);
  swv_printf("osc_cal = %04x\n", (int)err, (unsigned)osc_cal);
  err = VL53L0X_StaticInit(&dev);
  if (err != VL53L0X_ERROR_NONE) swv_printf("err3 = %d\n", (int)err);
  uint32_t ref_spad_cnt;
  uint8_t is_ap_spads;
  err = VL53L0X_PerformRefSpadManagement(&dev, &ref_spad_cnt, &is_ap_spads);
  if (err != VL53L0X_ERROR_NONE) swv_printf("err4 = %d\n", (int)err);
  swv_printf("ref_spad_cnt = %08x, is_ap_spads = %02x\n", (unsigned)ref_spad_cnt, (unsigned)is_ap_spads);
  uint8_t vhv_settings, phase_cal;
  err = VL53L0X_PerformRefCalibration(&dev, &vhv_settings, &phase_cal);
  if (err != VL53L0X_ERROR_NONE) swv_printf("err5 = %d\n", (int)err);
  swv_printf("vhv_settings = %02x, phase_cal = %02x\n", (unsigned)vhv_settings, (unsigned)phase_cal);

  err = VL53L0X_SetDeviceMode(&dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  if (err != VL53L0X_ERROR_NONE) swv_printf("err6 = %d\n", (int)err);
  err = VL53L0X_StartMeasurement(&dev);
  if (err != VL53L0X_ERROR_NONE) swv_printf("err7 = %d\n", (int)err);
  while (1) {
    uint8_t ready;
    do {
      VL53L0X_GetMeasurementDataReady(&dev, &ready);
    } while (!ready);
    VL53L0X_RangingMeasurementData_t meas;
    VL53L0X_Error err = VL53L0X_GetRangingMeasurementData(&dev, &meas);
    swv_printf("err = %d, range = %u mm / %u mm, rg status = %d\n",
      (int)err, (unsigned)meas.RangeMilliMeter, (unsigned)meas.RangeDMaxMilliMeter, (int)meas.RangeStatus);
    // err = 0, range = 400 mm / 1174 mm, rg status = 0
    // err = 0, range = 8190 mm / 1174 mm, rg status = 4
    HAL_Delay(100);

    uint16_t vol = audio_volume;
    int32_t target_vol = 1024 * (meas.RangeMilliMeter - 100) / (500 - 100);
    if (target_vol >= 1024) target_vol = 1024;
    else if (target_vol < 0) target_vol = 0;
    audio_volume = vol + (int32_t)(target_vol - vol) * 25 / 100;
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
  static uint32_t parity = 0;
  HAL_GPIO_WritePin(ACT_LED_PORT, ACT_LED_PIN, parity ^= 1);
}

void DMA1_Channel1_IRQHandler()
{
  HAL_DMA_IRQHandler(i2s1.hdmatx);
}
void SPI1_IRQHandler()
{
  HAL_I2S_IRQHandler(&i2s1);
}
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *i2s1)
{
  static int count = 0;
  // 128000 samples = 2.67 s
  if ((count = (count + 1) % 128) == 0) swv_printf("! half\n");

  audio_decode(0, false);
}
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *i2s1)
{
  static int count = 0;
  // 128000 samples = 2.67 s
  if ((count = (count + 1) % 128) == 0) swv_printf("! complete\n");

  audio_decode(1, false);
}

void DMA1_Channel2_3_IRQHandler()
{
  HAL_DMA_IRQHandler(spi2.hdmatx);
  HAL_DMA_IRQHandler(spi2.hdmarx);
}
void SPI2_IRQHandler()
{
  HAL_SPI_IRQHandler(&spi2);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi_handle)
{
  if (spi_handle == &spi2) {
    flash_read_dma_wait_end();
    audio_read_mark_end();
  }
}

// Platform layer for VL53L0X
// vl53l0x/vl53l0x_platform.h

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
  HAL_I2C_Mem_Write(&i2c2, 0x29 << 1, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 1000);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
  HAL_I2C_Mem_Read(&i2c2, 0x29 << 1, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 1000);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
  HAL_I2C_Mem_Write(&i2c2, 0x29 << 1, index, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
  data = __builtin_bswap16(data);
  HAL_I2C_Mem_Write(&i2c2, 0x29 << 1, index, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&data, 2, 1000);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
  data = __builtin_bswap32(data);
  HAL_I2C_Mem_Write(&i2c2, 0x29 << 1, index, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&data, 4, 1000);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
  HAL_I2C_Mem_Read(&i2c2, 0x29 << 1, index, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
  HAL_I2C_Mem_Read(&i2c2, 0x29 << 1, index, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, 2, 1000);
  *data = __builtin_bswap16(*data);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
  HAL_I2C_Mem_Read(&i2c2, 0x29 << 1, index, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, 4, 1000);
  *data = __builtin_bswap32(*data);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
  uint8_t x;
  HAL_I2C_Mem_Read(&i2c2, 0x29 << 1, index, I2C_MEMADD_SIZE_8BIT, &x, 1, 1000);
  x = (x & AndData) | OrData;
  HAL_I2C_Mem_Write(&i2c2, 0x29 << 1, index, I2C_MEMADD_SIZE_8BIT, &x, 1, 1000);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
  return VL53L0X_ERROR_NONE;
}
