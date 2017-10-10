/*
 * Copyright (c) 2014-2017 Cesanta Software Limited
 * All rights reserved
 */

#include "mgos_spi.h"

static struct mgos_spi *s_global_spi;

bool mgos_spi_init(void) {
  if (!mgos_sys_config_get_spi_enable()) return true;
  s_global_spi = mgos_spi_create(mgos_sys_config_get_spi());
  return (s_global_spi != NULL);
}

struct mgos_spi *mgos_spi_get_global(void) {
  return s_global_spi;
}
