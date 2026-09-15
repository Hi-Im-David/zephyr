/*
 * Dumps the nRF7120 APPROTECT boot trace over the console (uart20 on the DK).
 *
 * Drop this into your Zephyr application's source list - it is application
 * code, not MDK code, so do NOT place it under the patch/bsp/ tree or it will
 * be copied into nrfx and compiled without Zephyr headers.
 *
 * Requires the MDK to be built with NRF_ENABLE_NRF7120_BOOT_TRACE.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>

#include <mdk/nrf.h>
#include <mdk/nrf71/nrf71_boot_trace.h>

/* TAMPC signal CTRL layout: VALUE is bit 0, LOCK is bit 1, WRITEPROTECTION is
   bits 4..7. A locked signal cannot be changed until the next qualifying reset,
   which is the state the workaround exists to escape. */
#define SIG_VALUE(c)    (((c) >> 0) & 0x1u)
#define SIG_LOCK(c)     (((c) >> 1) & 0x1u)
#define SIG_WP(c)       (((c) >> 4) & 0xfu)

static void print_signal(const char *name, uint32_t ctrl)
{
    printk("      %-9s 0x%08x  value=%u lock=%u wp=0x%x%s\n",
           name, ctrl, SIG_VALUE(ctrl), SIG_LOCK(ctrl), SIG_WP(ctrl),
           SIG_LOCK(ctrl) ? "   <-- LOCKED" : "");
}

/* RESETREAS latches every cause since it was last cleared, so more than one bit
   is normal. */
static void print_resetreas(uint32_t v)
{
    static const char *const names[] = {
        "RESETPIN", "DOG0", "DOG1", "CTRLAPSOFT", "CTRLAPHARD", "CTRLAPPIN",
        "SREQ", "LOCKUP", "OFF", "LPCOMP", "DIF", "GRTC", "NFC", "SECTAMPER",
        "VBUS", NULL, "LMAC", "UMAC",
    };

    printk("      resetreas 0x%08x ", v);
    if (v == 0u) {
        printk("(none latched)");
    }
    for (uint32_t b = 0; b < ARRAY_SIZE(names); b++) {
        if ((v & (1u << b)) && names[b] != NULL) {
            printk("%s ", names[b]);
        }
    }
    printk("\n");
}

static void print_record(uint32_t n, const nrf71_boot_trace_record_t *r)
{
    uint32_t bootcount = (r->mramrecovery & REGULATORS_MRAMRECOVERY_BOOTCOUNT_Msk)
                         >> REGULATORS_MRAMRECOVERY_BOOTCOUNT_Pos;

    printk("  [%u] seq=%u bootcount=%u\n", n, r->seq, bootcount);
    print_resetreas(r->resetreas);
    printk("      rom report version=0x%08x stages=0x%08x%s\n",
           r->rom_report_version, r->rom_report_stages,
           (r->rom_report_version == 0u || r->rom_report_version == 0xFFFFFFFFu)
               ? "   <-- ROM never wrote the report" : "");

    printk("    entry (before the workaround touched anything):\n");
    print_signal("RESETBEH", r->entry_resetbehavior);
    print_signal("DBGEN",    r->entry_dbgen);
    print_signal("NIDEN",    r->entry_niden);
    print_signal("SPIDEN",   r->entry_spiden);
    print_signal("SPNIDEN",  r->entry_spniden);
    print_signal("AP0 DBGEN", r->entry_ap_dbgen);

    /* Judge against the value we asked for, not against the entry value - the
       register is often already armed on entry, in which case an unchanged
       readback means success rather than a rejected write. */
    printk("    after arming RESETBEHAVIOR:\n");
    printk("      RESETBEH  0x%08x  %s\n", r->armed_resetbehavior,
           SIG_VALUE(r->armed_resetbehavior)
               ? "armed"
               : "<-- NOT ARMED, TAMPC rejected the write");

    if (r->exit_valid) {
        printk("    exit (SystemInit completed):\n");
        print_signal("RESETBEH", r->exit_resetbehavior);
        print_signal("DBGEN",    r->exit_dbgen);
        print_signal("NIDEN",    r->exit_niden);
        print_signal("SPIDEN",   r->exit_spiden);
        print_signal("SPNIDEN",  r->exit_spniden);
        print_signal("AP0 DBGEN", r->exit_ap_dbgen);
    } else {
        printk("    exit: NOT REACHED - this pass either soft-reset itself or\n");
        printk("          hung before the end of SystemInit\n");
    }
}

void nrf71_boot_trace_dump(void)
{
    uint32_t count = nrf71_boot_trace_count();

    printk("\n==== nRF7120 boot trace ====\n");

    if (count == 0u) {
        /* Either a genuine cold boot, or the buffer was clobbered. Both look
           the same from here, which is why the magic check exists. */
        printk("  no valid records (cold boot, or .noinit was overwritten)\n");
        printk("============================\n\n");
        return;
    }

    printk("  %u record(s), oldest first, %u boot(s) since cold start\n",
           count, nrf71_boot_trace.total);

    for (uint32_t i = 0; i < count; i++) {
        print_record(i, nrf71_boot_trace_at(i));
    }

    printk("============================\n\n");
}

static int nrf71_boot_trace_dump_init(void)
{
    nrf71_boot_trace_dump();
    return 0;
}

/* APPLICATION level so the console driver is already up. */
SYS_INIT(nrf71_boot_trace_dump_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
