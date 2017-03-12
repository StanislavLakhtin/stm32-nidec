//
// Created by sl on 18.02.17.
//

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f1/nvic.h>
#include <pcd8544.h>
#include <stdio.h>
#include <errno.h>

#define MOTOR_PORTA GPIOA
#define MOTOR_A0 GPIO8
#define MOTOR_A1 GPIO9
#define MOTOR_PORTB GPIOA
#define MOTOR_B0 GPIO10
#define MOTOR_B1 GPIO11

static uint8_t halfStep[8][4] = {{1,0,0,0}, {1,0,1,0}, {0,0,1,0}, {0,1,1,0},
                                    {0,1,0,0}, {0,1,0,1},{0,0,0,1},{1,0,0,1}};
static uint8_t fullStep[4][4] = {{1,0,1,0}, {0,1,1,0}, {0,1,0,1}, {1,0,0,1}};

static void clock_setup(void) {
  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);

  /* Enable SPI2 Periph and gpio clocks */
  rcc_periph_clock_enable(RCC_SPI2);

  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_USART3);
}

static void pcd8544_setup(void) {
  /* Configure GPIOs: SS=PCD8544_SPI_SS, SCK=PCD8544_SPI_SCK, MISO=UNUSED and MOSI=PCD8544_SPI_MOSI */
  gpio_set_mode(PCD8544_SPI_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                PCD8544_SPI_MOSI | PCD8544_SPI_SCK | PCD8544_SPI_SS);
  /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
  spi_reset(PCD8544_SPI);
  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/64 of peripheral clock frequency
   * Clock: CPOL CPHA (0:0)
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
  spi_init_master(PCD8544_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(PCD8544_SPI);
  spi_set_nss_high(PCD8544_SPI);

  /* Enable SPI1 periph. */
  spi_enable(PCD8544_SPI);

  /* Configure GPIOs: DC, SCE, RST */
  gpio_set_mode(PCD8544_RST_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                PCD8544_RST);
  gpio_set_mode(PCD8544_DC_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                PCD8544_DC);
  gpio_set_mode(PCD8544_SCE_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                PCD8544_SCE);

  gpio_set(PCD8544_RST_PORT, PCD8544_RST);
  gpio_set(PCD8544_SCE_PORT, PCD8544_SCE);

  for (int i = 0; i < 200; i++)
      __asm__("nop");

}

static void gpio_setup(void) {
  /* Configure GPIOs */
  gpio_set_mode(MOTOR_PORTA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                MOTOR_A0 | MOTOR_A1);
  gpio_set_mode(MOTOR_PORTB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                MOTOR_B0 | MOTOR_B1);
  gpio_clear(MOTOR_PORTA, MOTOR_A0 | MOTOR_A1);
  gpio_clear(MOTOR_PORTB, MOTOR_B0 | MOTOR_B1);
}

int _write(int file, char *ptr, int len) {
  int i;

  if (file == 1) {
    for (i = 0; i < len; i++)
      usart_send_blocking(USART2, ptr[i]);
    return i;
  }
  errno = EIO;
  return -1;
}

static void usart_setup(void) {
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_USART2_RX);

  usart_set_baudrate(USART2, 9600);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX_RX);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_enable(USART2);
}

void motorPinToggle(uint8_t state, uint32_t port, uint16_t pin) {
  if (state)
    gpio_set(port, pin);
  else
    gpio_clear(port, pin);
}

void motorStep(uint8_t state[4]) {
  wchar_t buffer[50];
  for (int i=0; i<4; i++) {
    swprintf(buffer, 50, L"%d", state[i]);
    pcd8544_drawText(i*8, 0, BLACK, buffer);
  }
  motorPinToggle(state[0], MOTOR_PORTA, MOTOR_A0);
  motorPinToggle(state[1], MOTOR_PORTA, MOTOR_A1);
  motorPinToggle(state[2], MOTOR_PORTB, MOTOR_B0);
  motorPinToggle(state[3], MOTOR_PORTB, MOTOR_B1);
}

int main(void) {
  clock_setup();
  gpio_setup();
  pcd8544_setup();
  usart_setup();
  pcd8544_init();
  wchar_t buffer[50];
  int8_t index = 0;
  int8_t dirIndex = 1;
  int8_t steps = 0;

  while (1) {
    pcd8544_clearDisplay();
    swprintf(buffer, 50, L"%d", index);
    pcd8544_drawText(0, 40, BLACK, buffer);
    motorStep(halfStep[index]);
    pcd8544_display();
    index+=dirIndex;
    for (int i = 0; i < 1000000; i++)
        __asm__("nop");
    if (dirIndex && index>7 )
      index = 0;
    else if (index<0)
      index = 7;
    steps += dirIndex;
    if (steps>40) {
      steps = 40;
      dirIndex = dirIndex * -1;
    } else if (steps<0) {
      steps = 0;
      dirIndex = dirIndex * -1;
    }
  }
}

