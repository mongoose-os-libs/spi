/*
 * Copyright (c) 2014-2017 Cesanta Software Limited
 * All rights reserved
 */

#include "mgos_spi.h"

static struct mgos_spi *s_global_spi;

bool mgos_spi_init(void) {
  const struct sys_config_spi *cfg = &get_cfg()->spi;
  if (!cfg->enable) return true;
  s_global_spi = mgos_spi_create(cfg);
  return (s_global_spi != NULL);
}

struct mgos_spi *mgos_spi_get_global(void) {
  return s_global_spi;
}
