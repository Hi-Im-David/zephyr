/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This header is part of mspi_dw.c extracted only for clarity.
 * It is not supposed to be included by any file other than mspi_dw.c.
 */

#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_exmif)

#include <nrf.h>

static inline void vendor_specific_init(const struct device *dev, const struct mspi_dw_config * config)
{
	ARG_UNUSED(dev);

	NRF_EXMIF->EVENTS_CORE = 0;
	NRF_EXMIF->INTENSET = BIT(EXMIF_INTENSET_CORE_Pos);
}

static inline void vendor_specific_suspend(const struct device *dev, const struct mspi_dw_config * config)
{
	ARG_UNUSED(dev);

	NRF_EXMIF->TASKS_STOP = 1;
}

static inline void vendor_specific_resume(const struct device *dev, const struct mspi_dw_config * config)
{
	ARG_UNUSED(dev);

	NRF_EXMIF->TASKS_START = 1;

	/* Try to write an SSI register and wait until the write is successful
	 * to ensure that the clock that drives the SSI core is ready.
	 */
	uint32_t rxftlr = read_rxftlr(dev);
	uint32_t rxftlr_mod = rxftlr ^ 1;

	do {
		write_rxftlr(dev, rxftlr_mod);
		rxftlr = read_rxftlr(dev);
	} while (rxftlr != rxftlr_mod);
}

static inline void vendor_specific_irq_clear(const struct device *dev, const struct mspi_dw_config * config)
{
	ARG_UNUSED(dev);

	NRF_EXMIF->EVENTS_CORE = 0;
}

#if defined(CONFIG_MSPI_XIP)
static inline int vendor_specific_xip_enable(const struct device *dev, const struct mspi_dw_config * config,
					     const struct mspi_dev_id *dev_id,
					     const struct mspi_xip_cfg *cfg)
{
	ARG_UNUSED(dev);

	if (dev_id->dev_idx == 0) {
		NRF_EXMIF->EXTCONF1.OFFSET = cfg->address_offset;
		NRF_EXMIF->EXTCONF1.SIZE = cfg->address_offset
					 + cfg->size - 1;
		NRF_EXMIF->EXTCONF1.ENABLE = 1;
	} else if (dev_id->dev_idx == 1) {
		NRF_EXMIF->EXTCONF2.OFFSET = cfg->address_offset;
		NRF_EXMIF->EXTCONF2.SIZE = cfg->address_offset
					 + cfg->size - 1;
		NRF_EXMIF->EXTCONF2.ENABLE = 1;
	} else {
		return -EINVAL;
	}

	return 0;
}

static inline int vendor_specific_xip_disable(const struct device *dev, const struct mspi_dw_config * config,
					      const struct mspi_dev_id *dev_id,
					      const struct mspi_xip_cfg *cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);

	if (dev_id->dev_idx == 0) {
		NRF_EXMIF->EXTCONF1.ENABLE = 0;
	} else if (dev_id->dev_idx == 1) {
		NRF_EXMIF->EXTCONF2.ENABLE = 0;
	} else {
		return -EINVAL;
	}

	return 0;
}
#endif /* defined(CONFIG_MSPI_XIP) */

#endif /* DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_exmif) */


#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_qspi2_controller)

#include <nrf.h>
#include <zephyr/kernel.h>
// #include "mspi_dw.h"

static inline void vendor_specific_init(const struct device *dev, const struct mspi_dw_config * config)
{
	NRF_QSPI_Type *preg = (NRF_QSPI_Type *)config->wrapper_regs;
	// printk("initialising QSPI, 0x%08X\n", (int)preg);
	// ARG_UNUSED(dev);
	preg->ENABLE = 1; //5005c400
	preg->EVENTS_CORE = 0;
	preg->INTENSET = BIT(QSPI_INTENSET_CORE_Pos);
	
}

static inline void vendor_specific_suspend(const struct device *dev, const struct mspi_dw_config * config)
{
	// NRF_QSPI_Type *preg = (NRF_QSPI_Type *)config->wrapper_regs;
	ARG_UNUSED(dev);

	// preg->ENABLE = 0; 
	// preg->TASKS_STOP = 1;
}

static inline void vendor_specific_resume(const struct device *dev, const struct mspi_dw_config * config)
{
	// NRF_QSPI_Type *preg = (NRF_QSPI_Type *)config->wrapper_regs;
	ARG_UNUSED(dev);

	// preg->ENABLE = 1;
}

static inline void vendor_specific_irq_clear(const struct device *dev, const struct mspi_dw_config * config)
{
	// ARG_UNUSED(dev);
	NRF_QSPI_Type *preg = (NRF_QSPI_Type *)config->wrapper_regs;
	preg->EVENTS_CORE = 0;
}

static inline void vendor_specific_enable_dma(const struct device *dev, const struct mspi_dw_config * config)
{
	// ARG_UNUSED(dev);
	NRF_QSPI_Type *preg = (NRF_QSPI_Type *)config->wrapper_regs;
	preg->TASKS_START = 1;
}

#if defined(CONFIG_MSPI_XIP)
static inline int vendor_specific_xip_enable(const struct device *dev, const struct mspi_dw_config * config,
					     const struct mspi_dev_id *dev_id,
					     const struct mspi_xip_cfg *cfg)
{
	ARG_UNUSED(dev);

	if (dev_id->dev_idx == 0) {
		NRF_QSPI01->EXTCONF1.OFFSET = cfg->address_offset;
		NRF_QSPI01->EXTCONF1.SIZE = cfg->address_offset
					 + cfg->size - 1;
		NRF_QSPI01->EXTCONF1.ENABLE = 1;
	} else if (dev_id->dev_idx == 1) {
		NRF_QSPI01->EXTCONF2.OFFSET = cfg->address_offset;
		NRF_QSPI01->EXTCONF2.SIZE = cfg->address_offset
					 + cfg->size - 1;
		NRF_QSPI01->EXTCONF2.ENABLE = 1;
	} else {
		return -EINVAL;
	}

	return 0;
}

static inline int vendor_specific_xip_disable(const struct device *dev, const struct mspi_dw_config * config,
					      const struct mspi_dev_id *dev_id,
					      const struct mspi_xip_cfg *cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);

	if (dev_id->dev_idx == 0) {
		NRF_QSPI01->EXTCONF1.ENABLE = 0;
	} else if (dev_id->dev_idx == 1) {
		NRF_QSPI01->EXTCONF2.ENABLE = 0;
	} else {
		return -EINVAL;
	}

	return 0;
}
#endif /* defined(CONFIG_MSPI_XIP) */


#endif /* DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_exmif) */