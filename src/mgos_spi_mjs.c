/*
 * Copyright (c) 2014-2017 Cesanta Software Limited
 * All rights reserved
 */

#include "mgos_spi.h"

/*
 * Create a transaction struct and pre-fill it with the given cs, mode and
 * freq. When the structure is not needed anymore, client code should free()
 * it.
 *
 * This function is useful mostly for ffi.
 */
struct mgos_spi_txn *mgos_spi_create_txn(int cs, int mode, int freq) {
  struct mgos_spi_txn *txn = calloc(sizeof(*txn), 1);

  txn->cs = cs;
  txn->mode = mode;
  txn->freq = freq;

  return txn;
}

/*
 * Set half-duplex params on the transaction struct.
 *
 * This function is useful mostly for ffi.
 */
void mgos_spi_set_hd_txn(struct mgos_spi_txn *txn, int tx_len,
                         const void *tx_data, int dummy_len, int rx_len,
                         void *rx_data) {
  txn->hd.tx_len = tx_len;
  txn->hd.tx_data = tx_data;
  txn->hd.dummy_len = dummy_len;
  txn->hd.rx_len = rx_len;
  txn->hd.rx_data = rx_data;
}

/*
 * Set full-duplex params on the transaction struct.
 *
 * This function is useful mostly for ffi.
 */
void mgos_spi_set_fd_txn(struct mgos_spi_txn *txn, int len, const void *tx_data,
                         void *rx_data) {
  txn->fd.len = len;
  txn->fd.tx_data = tx_data;
  txn->fd.rx_data = rx_data;
}
