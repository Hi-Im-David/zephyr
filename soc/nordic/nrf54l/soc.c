/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for Nordic Semiconductor nRF54L family processor
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Nordic Semiconductor nRF54L family processor.
 */

/* Include autoconf for cases when this file is used in special build (e.g. TFM) */
#include <zephyr/autoconf.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/cache.h>
#include <soc/nrfx_coredep.h>
#include <system_nrf54l.h>
#include <soc.h>
LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

#if (defined(NRF_APPLICATION) && !defined(CONFIG_TRUSTED_EXECUTION_NONSECURE)) || \
	!defined(__ZEPHYR__)

#include <nrf_erratas.h>
#include <hal/nrf_glitchdet.h>
#include <hal/nrf_power.h>
#include <hal/nrf_regulators.h>
#include <zephyr/dt-bindings/regulator/nrf5x.h>
#include <soc_nrf_oscillator.h>


static inline void power_and_clock_configuration(void)
{
/* NRF_REGULATORS and NRF_OSCILLATORS are configured to be secure
 * as NRF_REGULATORS.POFCON is needed by the secure image to
 * prevent glitches when the power supply is attacked.
 *
 * NRF_OSCILLATORS is also configured as secure because of a HW limitation
 * that requires them to be configured with the same security property.
 */
	setup_lfxo_capacitors();
	setup_hfxo_capacitors();

	if (IS_ENABLED(CONFIG_SOC_NRF_FORCE_CONSTLAT)) {
		nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_CONSTLAT);
	}

#if (DT_PROP(DT_NODELABEL(vregmain), regulator_initial_mode) == NRF5X_REG_MODE_DCDC)
#if NRF54L_ERRATA_31_ENABLE_WORKAROUND
	/* Workaround for Errata 31 */
	if (nrf54l_errata_31()) {
		*((volatile uint32_t *)0x50120624ul) = 20 | 1<<5;
		*((volatile uint32_t *)0x5012063Cul) &= ~(1<<19);
	}
#endif
	nrf_regulators_vreg_enable_set(NRF_REGULATORS, NRF_REGULATORS_VREG_MAIN, true);
#endif

}
#endif /* NRF_APPLICATION && !CONFIG_TRUSTED_EXECUTION_NONSECURE */

int nordicsemi_nrf54l_init(void)
{
	/* Update the SystemCoreClock global variable with current core clock
	 * retrieved from the DT.
	 */
	SystemCoreClock = NRF_PERIPH_GET_FREQUENCY(DT_NODELABEL(cpu));

	sys_cache_instr_enable();

#if (defined(NRF_APPLICATION) && !defined(CONFIG_TRUSTED_EXECUTION_NONSECURE)) || \
	!defined(__ZEPHYR__)
	power_and_clock_configuration();
#endif

	return 0;
}

void arch_busy_wait(uint32_t time_us)
{
	nrfx_coredep_delay_us(time_us);
}

SYS_INIT(nordicsemi_nrf54l_init, PRE_KERNEL_1, 0);
