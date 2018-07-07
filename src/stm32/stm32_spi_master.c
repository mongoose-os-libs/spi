/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the ""License"");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mgos_spi.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "common/cs_dbg.h"

#include "mgos_gpio.h"
#include "mgos_system.h"
#include "mgos_sys_config.h"

#include "stm32_sdk_hal.h"

struct mgos_spi {
  int unit_no;
  int freq;
  int sclk_gpio;
  int cs_gpio[3];
  volatile SPI_TypeDef *regs;
  unsigned int mode : 2;
  unsigned int debug : 1;
};

struct mgos_spi *mgos_spi_create(const struct mgos_config_spi *cfg) {
  struct mgos_spi *c = (struct mgos_spi *) calloc(1, sizeof(*c));
  if (c == NULL) goto out_err;

  switch (cfg->unit_no) {
    case 1:
      c->regs = SPI1;
      __HAL_RCC_SPI1_CLK_ENABLE();
      __HAL_RCC_SPI1_FORCE_RESET();
      __HAL_RCC_SPI1_RELEASE_RESET();
      break;
    case 2:
      c->regs = SPI2;
      __HAL_RCC_SPI2_CLK_ENABLE();
      __HAL_RCC_SPI2_FORCE_RESET();
      __HAL_RCC_SPI2_RELEASE_RESET();
      break;
    case 3:
      c->regs = SPI3;
      __HAL_RCC_SPI3_CLK_ENABLE();
      __HAL_RCC_SPI3_FORCE_RESET();
      __HAL_RCC_SPI3_RELEASE_RESET();
      break;
    case 4:
      c->regs = SPI4;
      __HAL_RCC_SPI4_CLK_ENABLE();
      __HAL_RCC_SPI4_FORCE_RESET();
      __HAL_RCC_SPI4_RELEASE_RESET();
      break;
    case 5:
      c->regs = SPI5;
      __HAL_RCC_SPI5_CLK_ENABLE();
      __HAL_RCC_SPI5_FORCE_RESET();
      __HAL_RCC_SPI5_RELEASE_RESET();
      break;
#ifdef SPI6
    case 6:
      c->regs = SPI6;
      __HAL_RCC_SPI6_CLK_ENABLE();
      __HAL_RCC_SPI6_FORCE_RESET();
      __HAL_RCC_SPI6_RELEASE_RESET();
      break;
#endif
    default:
      LOG(LL_ERROR, ("Invalid unit_no %d", cfg->unit_no));
      goto out_err;
  }
  c->unit_no = cfg->unit_no;

  if (cfg->sclk_gpio < 0 || (cfg->miso_gpio < 0 && cfg->mosi_gpio < 0)) {
    LOG(LL_ERROR, ("Invalid pin settings %d %d %d", cfg->sclk_gpio,
                   cfg->miso_gpio, cfg->mosi_gpio));
    goto out_err;
  }

  GPIO_InitTypeDef gs = {
      .Mode = GPIO_MODE_AF_PP,
      .Pull = GPIO_PULLUP,
      .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };

  if (cfg->miso_gpio >= 0) {
    gs.Pin = STM32_PIN_MASK(cfg->miso_gpio);
    gs.Alternate = STM32_PIN_AF(cfg->miso_gpio);
    HAL_GPIO_Init(stm32_gpio_port_base(cfg->miso_gpio), &gs);
  }
  if (cfg->mosi_gpio >= 0) {
    gs.Pin = STM32_PIN_MASK(cfg->mosi_gpio);
    gs.Alternate = STM32_PIN_AF(cfg->mosi_gpio);
    HAL_GPIO_Init(stm32_gpio_port_base(cfg->mosi_gpio), &gs);
  }
  if (cfg->sclk_gpio >= 0) {
    c->sclk_gpio = cfg->sclk_gpio;
    gs.Pin = STM32_PIN_MASK(cfg->sclk_gpio);
    gs.Alternate = STM32_PIN_AF(cfg->sclk_gpio);
    HAL_GPIO_Init(stm32_gpio_port_base(cfg->sclk_gpio), &gs);
  }
  gs.Alternate = 0;  // GPIO
  gs.Mode = GPIO_MODE_OUTPUT_PP;
  gs.Pull = GPIO_PULLUP;
  c->cs_gpio[0] = cfg->cs0_gpio;
  if (cfg->cs0_gpio >= 0) {
    mgos_gpio_write(cfg->cs0_gpio, 1);
    gs.Pin = STM32_PIN_MASK(cfg->cs0_gpio);
    HAL_GPIO_Init(stm32_gpio_port_base(cfg->cs0_gpio), &gs);
  }
  c->cs_gpio[1] = cfg->cs1_gpio;
  if (cfg->cs1_gpio >= 0) {
    mgos_gpio_write(cfg->cs2_gpio, 1);
    gs.Pin = STM32_PIN_MASK(cfg->cs1_gpio);
    HAL_GPIO_Init(stm32_gpio_port_base(cfg->cs0_gpio), &gs);
  }
  c->cs_gpio[2] = cfg->cs2_gpio;
  if (cfg->cs2_gpio >= 0) {
    mgos_gpio_write(cfg->cs2_gpio, 1);
    gs.Pin = STM32_PIN_MASK(cfg->cs2_gpio);
    HAL_GPIO_Init(stm32_gpio_port_base(cfg->cs0_gpio), &gs);
  }

  if (!mgos_spi_configure(c, cfg)) {
    goto out_err;
  }

  char b1[8], b2[8], b3[8], b4[8], b5[8], b6[8];
  LOG(LL_INFO,
      ("SPI%d init ok (MISO: %s, MOSI: %s, SCLK: %s; "
       "CS0/1/2: %s/%s/%s)",
       cfg->unit_no, mgos_gpio_str(cfg->miso_gpio, b1),
       mgos_gpio_str(cfg->mosi_gpio, b2), mgos_gpio_str(cfg->sclk_gpio, b3),
       mgos_gpio_str(cfg->cs0_gpio, b4), mgos_gpio_str(cfg->cs1_gpio, b5),
       mgos_gpio_str(cfg->cs2_gpio, b6)));
  return c;

out_err:
  free(c);
  LOG(LL_ERROR, ("Invalid SPI settings"));
  return NULL;
}

bool mgos_spi_configure(struct mgos_spi *c, const struct mgos_config_spi *cfg) {
  /* Reset everything and disable. Enable manual SS control. */
  c->regs->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
  c->regs->CR2 = 0;
  c->debug = cfg->debug;
  return true;
}

static bool stm32_spi_set_freq(struct mgos_spi *c, int freq) {
  if (c->freq == freq) return true;
  CLEAR_BIT(c->regs->CR1, SPI_CR1_SPE);
  int eff_freq;
  switch (c->unit_no) {
    case 1:
    case 4:
    case 5:
    case 6:
      eff_freq = HAL_RCC_GetPCLK2Freq();
      break;
    case 2:
    case 3:
      eff_freq = HAL_RCC_GetPCLK1Freq();
      break;
    default:
      return false;
  }
  uint32_t br = 0;
  eff_freq /= 2;
  while (eff_freq > freq) {
    br++;
    eff_freq /= 2;
    if (br > 7) return false;
  }
  MODIFY_REG(c->regs->CR1, SPI_CR1_BR_Msk, br << SPI_CR1_BR_Pos);
  c->freq = freq;
  if (c->debug) {
    LOG(LL_DEBUG, ("freq %d => div %d (br %d) => eff_freq %d", c->freq,
                   (int) (1 << (br + 1)), (int) br, eff_freq));
  }
  return true;
}

static bool stm32_spi_set_mode(struct mgos_spi *c, int mode) {
  CLEAR_BIT(c->regs->CR1, SPI_CR1_CPOL | SPI_CR1_CPHA);
  switch (mode) {
    case 0:
      mgos_gpio_set_pull(c->sclk_gpio, MGOS_GPIO_PULL_DOWN);
      break;
    case 1:
      mgos_gpio_set_pull(c->sclk_gpio, MGOS_GPIO_PULL_DOWN);
      SET_BIT(c->regs->CR1, SPI_CR1_CPHA);
      break;
    case 2:
      mgos_gpio_set_pull(c->sclk_gpio, MGOS_GPIO_PULL_UP);
      SET_BIT(c->regs->CR1, SPI_CR1_CPOL);
      break;
    case 3:
      mgos_gpio_set_pull(c->sclk_gpio, MGOS_GPIO_PULL_UP);
      SET_BIT(c->regs->CR1, SPI_CR1_CPOL | SPI_CR1_CPHA);
      break;
    default:
      return false;
  }
  return true;
}

inline static void stm32_spi_wait_tx_empty(struct mgos_spi *c) {
  while (!(c->regs->SR & SPI_SR_TXE)) {
  }
}

inline static void stm32_spi_wait_tx_idle(struct mgos_spi *c) {
  stm32_spi_wait_tx_empty(c);
  while (c->regs->SR & SPI_SR_BSY) {
  }
}

static bool stm32_spi_run_txn_fd(struct mgos_spi *c,
                                 const struct mgos_spi_txn *txn) {
  size_t len = txn->fd.len;
  const uint8_t *tx_data = (const uint8_t *) txn->hd.tx_data;
  uint8_t *rx_data = (uint8_t *) txn->fd.rx_data;
  if (c->debug) {
    LOG(LL_DEBUG, ("len %d", (int) len));
  }

  /* Clear MODF error, if any, by reading SR. */;
  (void) c->regs->SR;
  /* Enable SPI in master mode with software SS control
   * (at this point CSx is already asserted). */
  SET_BIT(c->regs->CR1, SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE);

  uint8_t byte = c->regs->DR; /* Clear the OVR flag (if any). */
  while (len > 0) {
    byte = *tx_data++;
    stm32_spi_wait_tx_empty(c);
    if (c->debug) LOG(LL_DEBUG, ("write 0x%02x", byte));
    c->regs->DR = byte;
    while (!(c->regs->SR & SPI_SR_RXNE)) {
    }
    byte = c->regs->DR;
    if (c->debug) LOG(LL_DEBUG, ("read 0x%02x", byte));
    *rx_data++ = byte;
    len--;
  }

  return true;
}

static bool stm32_spi_run_txn_hd(struct mgos_spi *c,
                                 const struct mgos_spi_txn *txn) {
  const uint8_t *tx_data = (const uint8_t *) txn->hd.tx_data;
  size_t tx_len = txn->hd.tx_len;
  size_t dummy_len = txn->hd.dummy_len;
  uint8_t *rx_data = (uint8_t *) txn->hd.rx_data;
  size_t rx_len = txn->hd.rx_len;
  if (c->debug) {
    LOG(LL_DEBUG, ("tx_len %d dummy_len %d rx_len %d", (int) tx_len,
                   (int) dummy_len, (int) rx_len));
  }

  /* Clear MODF error, if any, by reading SR. */;
  (void) c->regs->SR;
  /* Enable SPI in master mode with software SS control
   * (at this point CSx is already asserted). */
  SET_BIT(c->regs->CR1, SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE);

  uint8_t byte;
  while (tx_len > 0) {
    byte = *tx_data++;
    stm32_spi_wait_tx_empty(c);
    if (c->debug) LOG(LL_DEBUG, ("write 0x%02x", byte));
    c->regs->DR = byte;
    tx_len--;
  }

  while (dummy_len > 0) {
    /* Clock is only output when there's data to transmit so we send out 0s. */
    stm32_spi_wait_tx_empty(c);
    c->regs->DR = 0;
    dummy_len--;
  }

  if (rx_len > 0) {
    /* Wait for tx_data and dummy bytes to finish transmitting. */
    stm32_spi_wait_tx_idle(c);
    /* Clear the OVR flag. */
    byte = c->regs->DR;
    do {
      stm32_spi_wait_tx_empty(c);
      /* ummy data to provide clock. */
      c->regs->DR = 0;
      while (!(c->regs->SR & SPI_SR_RXNE)) {
      }
      byte = c->regs->DR;
      if (c->debug) LOG(LL_DEBUG, ("read 0x%02x", byte));
      *rx_data++ = byte;
      rx_len--;
    } while (rx_len > 0);
  }

  stm32_spi_wait_tx_idle(c);

  return true;
}

bool mgos_spi_run_txn(struct mgos_spi *c, bool full_duplex,
                      const struct mgos_spi_txn *txn) {
  bool ret = false;
  int cs_gpio = -1;
  if (txn->cs >= 0) {
    if (txn->cs > 2) return false;
    cs_gpio = c->cs_gpio[txn->cs];
    if (cs_gpio < 0) return false;
  }
  if (txn->freq > 0 && !stm32_spi_set_freq(c, txn->freq)) {
    return false;
  }
  if (!stm32_spi_set_mode(c, txn->mode)) {
    return false;
  }
  if (cs_gpio > 0) {
    mgos_gpio_write(cs_gpio, 0);
  }
  if (full_duplex) {
    ret = stm32_spi_run_txn_fd(c, txn);
  } else {
    ret = stm32_spi_run_txn_hd(c, txn);
  }
  if (cs_gpio > 0) {
    mgos_gpio_write(cs_gpio, 1);
  }
  CLEAR_BIT(c->regs->CR1, SPI_CR1_SPE);
  return ret;
}

void mgos_spi_close(struct mgos_spi *c) {
  free(c);
}
