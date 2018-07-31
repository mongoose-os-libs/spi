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

/* Special unit number that means QSPI */
#define STM32_QSPI_UNIT_NO 128

struct mgos_spi {
  int unit_no;
  int freq;
  int sclk_gpio;
  int cs_gpio[3];
  volatile SPI_TypeDef *regs;
  volatile QUADSPI_TypeDef *qregs;
  volatile uint32_t *apb_en_reg;
  uint32_t apb_en_bit;
  unsigned int mode : 2;
  unsigned int debug : 1;
};

struct mgos_spi *mgos_spi_create(const struct mgos_config_spi *cfg) {
  struct mgos_spi *c = (struct mgos_spi *) calloc(1, sizeof(*c));
  if (c == NULL) goto out_err;
  volatile uint32_t *apb_en_reg, *apb_rst_reg;
  uint32_t apb_en_bit, apb_rst_bit;

  switch (cfg->unit_no) {
    case 1:
      c->regs = SPI1;
#ifdef RCC_APB1ENR_SPI1EN
      apb_en_reg = &RCC->APB1ENR;
      apb_en_bit = RCC_APB1ENR_SPI1EN;
      apb_rst_reg = &RCC->APB1RSTR;
      apb_rst_bit = RCC_APB1RSTR_SPI1RST;
#else
      apb_en_reg = &RCC->APB2ENR;
      apb_en_bit = RCC_APB2ENR_SPI1EN;
      apb_rst_reg = &RCC->APB2RSTR;
      apb_rst_bit = RCC_APB2RSTR_SPI1RST;
#endif
      break;
    case 2:
      c->regs = SPI2;
#ifdef RCC_APB1ENR_SPI2EN
      apb_en_reg = &RCC->APB1ENR;
      apb_en_bit = RCC_APB1ENR_SPI2EN;
      apb_rst_reg = &RCC->APB1RSTR;
      apb_rst_bit = RCC_APB1RSTR_SPI2RST;
#else
      apb_en_reg = &RCC->APB2ENR;
      apb_en_bit = RCC_APB2ENR_SPI2EN;
      apb_rst_reg = &RCC->APB2RSTR;
      apb_rst_bit = RCC_APB2RSTR_SPI2RST;
#endif
      break;
    case 3:
      c->regs = SPI3;
#ifdef RCC_APB1ENR_SPI3EN
      apb_en_reg = &RCC->APB1ENR;
      apb_en_bit = RCC_APB1ENR_SPI3EN;
      apb_rst_reg = &RCC->APB1RSTR;
      apb_rst_bit = RCC_APB1RSTR_SPI3RST;
#else
      apb_en_reg = &RCC->APB2ENR;
      apb_en_bit = RCC_APB2ENR_SPI3EN;
      apb_rst_reg = &RCC->APB2RSTR;
      apb_rst_bit = RCC_APB2RSTR_SPI3RST;
#endif
      break;
    case 4:
      c->regs = SPI4;
#ifdef RCC_APB1ENR_SPI4EN
      apb_en_reg = &RCC->APB1ENR;
      apb_en_bit = RCC_APB1ENR_SPI4EN;
      apb_rst_reg = &RCC->APB1RSTR;
      apb_rst_bit = RCC_APB1RSTR_SPI4RST;
#else
      apb_en_reg = &RCC->APB2ENR;
      apb_en_bit = RCC_APB2ENR_SPI4EN;
      apb_rst_reg = &RCC->APB2RSTR;
      apb_rst_bit = RCC_APB2RSTR_SPI4RST;
#endif
      break;
    case 5:
      c->regs = SPI5;
#ifdef RCC_APB1ENR_SPI5EN
      apb_en_reg = &RCC->APB1ENR;
      apb_en_bit = RCC_APB1ENR_SPI5EN;
      apb_rst_reg = &RCC->APB1RSTR;
      apb_rst_bit = RCC_APB1RSTR_SPI5RST;
#else
      apb_en_reg = &RCC->APB2ENR;
      apb_en_bit = RCC_APB2ENR_SPI5EN;
      apb_rst_reg = &RCC->APB2RSTR;
      apb_rst_bit = RCC_APB2RSTR_SPI5RST;
#endif
      break;
#ifdef SPI6
    case 6:
      c->regs = SPI6;
#ifdef RCC_APB1ENR_SPI6EN
      apb_en_reg = &RCC->APB1ENR;
      apb_en_bit = RCC_APB1ENR_SPI6EN;
      apb_rst_reg = &RCC->APB1RSTR;
      apb_rst_bit = RCC_APB1RSTR_SPI6RST;
#else
      apb_en_reg = &RCC->APB2ENR;
      apb_en_bit = RCC_APB2ENR_SPI6EN;
      apb_rst_reg = &RCC->APB2RSTR;
      apb_rst_bit = RCC_APB2RSTR_SPI6RST;
#endif
      break;
#endif
    case STM32_QSPI_UNIT_NO:
      c->qregs = QUADSPI;
      apb_en_reg = &RCC->AHB3ENR;
      apb_en_bit = RCC_AHB3ENR_QSPIEN;
      apb_rst_reg = &RCC->AHB3RSTR;
      apb_rst_bit = RCC_AHB3RSTR_QSPIRST;
      break;
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

  *apb_en_reg |= apb_en_bit;
  *apb_rst_reg |= apb_rst_bit;
  c->apb_en_reg = apb_en_reg;
  c->apb_en_bit = apb_en_bit;
  *apb_rst_reg &= ~apb_rst_bit;

  if (cfg->miso_gpio >= 0) {
    mgos_gpio_set_mode(cfg->miso_gpio, MGOS_GPIO_MODE_INPUT);
    mgos_gpio_set_pull(cfg->miso_gpio, MGOS_GPIO_PULL_UP);
  }
  if (cfg->mosi_gpio >= 0) {
    mgos_gpio_set_mode(cfg->mosi_gpio, MGOS_GPIO_MODE_OUTPUT);
  }
  if (cfg->sclk_gpio >= 0) {
    mgos_gpio_set_mode(cfg->sclk_gpio, MGOS_GPIO_MODE_OUTPUT);
    c->sclk_gpio = cfg->sclk_gpio;
  }
  c->cs_gpio[0] = cfg->cs0_gpio;
  if (cfg->cs0_gpio >= 0) {
    mgos_gpio_set_mode(cfg->cs0_gpio, MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_write(cfg->cs0_gpio, 1);
  }
  c->cs_gpio[1] = cfg->cs1_gpio;
  if (cfg->cs1_gpio >= 0) {
    mgos_gpio_set_mode(cfg->cs1_gpio, MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_write(cfg->cs1_gpio, 1);
  }
  c->cs_gpio[2] = cfg->cs2_gpio;
  if (cfg->cs2_gpio >= 0) {
    mgos_gpio_set_mode(cfg->cs2_gpio, MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_write(cfg->cs2_gpio, 1);
  }
  if (c->unit_no == STM32_QSPI_UNIT_NO) {
    if (cfg->qspi_io2 >= 0) {
      mgos_gpio_set_mode(cfg->qspi_io2, MGOS_GPIO_MODE_OUTPUT);
    }
    if (cfg->qspi_io3 >= 0) {
      mgos_gpio_set_mode(cfg->qspi_io3, MGOS_GPIO_MODE_OUTPUT);
    }
  }

  if (!mgos_spi_configure(c, cfg)) {
    goto out_err;
  }

  char b1[8], b2[8], b3[8], b4[8], b5[8], b6[8];
  LOG(LL_INFO,
      ("%sSPI%d init ok (MISO: %s, MOSI: %s, SCLK: %s; "
       "CS0/1/2: %s/%s/%s)",
       (cfg->unit_no == STM32_QSPI_UNIT_NO ? "Q" : ""), (cfg->unit_no & 0x7f),
       mgos_gpio_str(cfg->miso_gpio, b1), mgos_gpio_str(cfg->mosi_gpio, b2),
       mgos_gpio_str(cfg->sclk_gpio, b3), mgos_gpio_str(cfg->cs0_gpio, b4),
       mgos_gpio_str(cfg->cs1_gpio, b5), mgos_gpio_str(cfg->cs2_gpio, b6)));
  (void) b1;
  (void) b2;
  (void) b3;
  (void) b4;
  (void) b5;
  (void) b6;
  return c;

out_err:
  free(c);
  LOG(LL_ERROR, ("Invalid SPI settings"));
  return NULL;
}

static inline bool is_qspi(const struct mgos_spi *c) {
  return (c->unit_no == STM32_QSPI_UNIT_NO);
}

bool mgos_spi_configure(struct mgos_spi *c, const struct mgos_config_spi *cfg) {
  if (!is_qspi(c)) {
    /* Reset everything and disable. Enable manual SS control. */
    c->regs->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    c->regs->CR2 = 0;
  } else {
    c->qregs->CR = 0;
    c->qregs->DCR = 0;
    /* We do not use address phase in our transactions but still need to set
     * chip size or controller raises invalid address error. */
    c->qregs->DCR = (31 << QUADSPI_DCR_FSIZE_Pos);
  }
  c->debug = cfg->debug;
  return true;
}

static bool stm32_qspi_set_freq(struct mgos_spi *c, int freq) {
  CLEAR_BIT(c->qregs->CR, QUADSPI_CR_EN);
  int eff_freq = HAL_RCC_GetHCLKFreq();
  int div = eff_freq / freq;
  if (eff_freq / div > freq) div++;
  if (div > 256) return false;
  eff_freq = eff_freq / div;
  uint32_t br = (uint32_t) div - 1;
  MODIFY_REG(c->qregs->CR, QUADSPI_CR_PRESCALER_Msk,
             br << QUADSPI_CR_PRESCALER_Pos);
  c->freq = freq;
  if (c->debug) {
    LOG(LL_DEBUG, ("freq %d => div %d (br %d) => eff_freq %d", c->freq, div,
                   (int) br, eff_freq));
  }
  return true;
}

static bool stm32_spi_set_freq(struct mgos_spi *c, int freq) {
  if (c->freq == freq) return true;
  if (is_qspi(c)) return stm32_qspi_set_freq(c, freq);
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
  uint32_t br = 0, div = 0;
  eff_freq /= 2;
  while (eff_freq > freq) {
    br++;
    eff_freq /= 2;
    if (br > 7) return false;
  }
  MODIFY_REG(c->regs->CR1, SPI_CR1_BR_Msk, br << SPI_CR1_BR_Pos);
  div = (1 << (br + 1));
  c->freq = freq;
  if (c->debug) {
    LOG(LL_DEBUG, ("freq %d => div %d (br %d) => eff_freq %d", c->freq,
                   (int) div, (int) br, eff_freq));
  }
  return true;
}

static bool stm32_qspi_set_mode(struct mgos_spi *c, int mode) {
  if (mode == 0) {
    CLEAR_BIT(c->qregs->DCR, QUADSPI_DCR_CKMODE);
  } else if (mode == 3) {
    SET_BIT(c->qregs->DCR, QUADSPI_DCR_CKMODE);
  } else {
    return false;
  }
  return true;
}

static bool stm32_spi_set_mode(struct mgos_spi *c, int mode) {
  if (is_qspi(c)) return stm32_qspi_set_mode(c, mode);
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
      /* Dummy data to provide clock. */
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

static inline uint32_t qspi_fifo_len(const struct mgos_spi *c) {
  return ((c->qregs->SR & QUADSPI_SR_FLEVEL_Msk) >> QUADSPI_SR_FLEVEL_Pos);
}

static bool stm32_qspi_run_txn_hd(struct mgos_spi *c,
                                  const struct mgos_spi_txn *txn) {
  const uint8_t *tx_data = (const uint8_t *) txn->hd.tx_data;
  size_t tx_len = txn->hd.tx_len;
  size_t dummy_len = txn->hd.dummy_len;
  uint8_t *rx_data = (uint8_t *) txn->hd.rx_data;
  size_t rx_len = txn->hd.rx_len;
  volatile uint8_t *drp = (volatile uint8_t *) &c->qregs->DR;
  if (c->debug) {
    LOG(LL_DEBUG, ("tx_len %d dummy_len %d rx_len %d", (int) tx_len,
                   (int) dummy_len, (int) rx_len));
  }
  SET_BIT(c->qregs->CR, QUADSPI_CR_EN);
  if (tx_len > 0) {
    c->qregs->FCR = QUADSPI_FCR_CTCF;
    /* Indirect write fmode (0), data phase only, single line dmode (1). */
    c->qregs->DLR = tx_len + dummy_len - 1;
    c->qregs->CCR = QSPI_DATA_1_LINE;
    while (tx_len > 0) {
      /* To avoid blocking the CPU we avoid filling up the FIFO */
      while (qspi_fifo_len(c) > 30) {
      }
      *drp = *tx_data++;
      tx_len--;
    }
    while (dummy_len > 0) {
      while (qspi_fifo_len(c) > 30) {
      }
      *drp = 0;
      dummy_len--;
    }
    while (!(c->qregs->SR & QUADSPI_SR_TCF)) {
    }
  }
  if (rx_len > 0) {
    c->qregs->FCR = QUADSPI_FCR_CTCF;
    /* Indirect read fmode (1), data phase only, single line dmode (1). */
    c->qregs->DLR = rx_len - 1;
    c->qregs->CCR = ((1 << QUADSPI_CCR_FMODE_Pos) | QSPI_DATA_1_LINE);
    while (rx_len > 0) {
      /* To avoid blocking the CPU we wait for data to become available. */
      while (qspi_fifo_len(c) == 0) {
      }
      *rx_data++ = *drp;
      rx_len--;
    }
  }
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
    ret = (is_qspi(c) ? false : stm32_spi_run_txn_fd(c, txn));
  } else {
    ret = (is_qspi(c) ? stm32_qspi_run_txn_hd(c, txn)
                      : stm32_spi_run_txn_hd(c, txn));
  }
  if (cs_gpio > 0) {
    mgos_gpio_write(cs_gpio, 1);
  }
  if (is_qspi(c)) {
    CLEAR_BIT(c->regs->CR1, SPI_CR1_SPE);
  } else {
    CLEAR_BIT(c->qregs->CR, QUADSPI_CR_EN);
  }
  return ret;
}

void mgos_spi_close(struct mgos_spi *c) {
  if (c == NULL) return;
  *c->apb_en_reg &= ~c->apb_en_bit;
  free(c);
}
