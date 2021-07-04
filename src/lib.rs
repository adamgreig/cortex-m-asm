// Many of these intrinsics are copied from
// https://github.com/rust-embedded/cortex-m/blob/master/asm/inline.rs
// Please refer to that repository for original authorship.

#![no_std]

nightly_crimes::nightly_crimes! {

#![feature(asm)]

use core::sync::atomic::{compiler_fence, Ordering};

#[inline(always)]
pub unsafe fn bkpt() {
    asm!("bkpt");
}

#[inline(always)]
pub unsafe fn control_r() -> u32 {
    let r;
    asm!("mrs {}, CONTROL", out(reg) r);
    r
}

#[inline(always)]
pub unsafe fn control_w(w: u32) {
    // ISB is required after writing to CONTROL,
    // per ARM architectural requirements (see Application Note 321).
    asm!(
        "msr CONTROL, {}",
        "isb",
        in(reg) w
    );

    // Ensure memory accesses are not reordered around the CONTROL update.
    compiler_fence(Ordering::SeqCst);
}

#[inline(always)]
pub unsafe fn cpsid() {
    asm!("cpsid i");

    // Ensure no subsequent memory accesses are reordered to before interrupts are disabled.
    compiler_fence(Ordering::SeqCst);
}

#[inline(always)]
pub unsafe fn cpsie() {
    // Ensure no preceeding memory accesses are reordered to after interrupts are enabled.
    compiler_fence(Ordering::SeqCst);

    asm!("cpsie i");
}

#[inline(always)]
pub unsafe fn delay(cyc: u32) {
    // The loop will normally take 3 to 4 CPU cycles per iteration, but superscalar cores
    // (eg. Cortex-M7) can potentially do it in 2, so we use that as the lower bound, since delaying
    // for more cycles is okay.
    // Add 1 to prevent an integer underflow which would cause a long freeze
    let real_cyc = 1 + cyc / 2;
    asm!(
        // Use local labels to avoid R_ARM_THM_JUMP8 relocations which fail on thumbv6m.
        "1:",
        "subs {}, #1",
        "bne 1b",
        inout(reg) real_cyc => _
    );
}

#[inline(always)]
pub unsafe fn dmb() {
    compiler_fence(Ordering::SeqCst);
    asm!("dmb");
    compiler_fence(Ordering::SeqCst);
}

#[inline(always)]
pub unsafe fn dsb() {
    compiler_fence(Ordering::SeqCst);
    asm!("dsb");
    compiler_fence(Ordering::SeqCst);
}

#[inline(always)]
pub unsafe fn isb() {
    compiler_fence(Ordering::SeqCst);
    asm!("isb");
    compiler_fence(Ordering::SeqCst);
}

#[inline(always)]
pub unsafe fn msp_r() -> u32 {
    let r;
    asm!("mrs {}, MSP", out(reg) r);
    r
}

#[inline(always)]
pub unsafe fn msp_w(val: u32) {
    asm!("msr MSP, {}", in(reg) val);
}

#[inline(always)]
pub unsafe fn apsr_r() -> u32 {
    let r;
    asm!("mrs {}, APSR", out(reg) r);
    r
}

#[inline(always)]
pub unsafe fn nop() {
    // NOTE: This is a `pure` asm block, but applying that option allows the compiler to eliminate
    // the nop entirely (or to collapse multiple subsequent ones). Since the user probably wants N
    // nops when they call `nop` N times, let's not add that option.
    asm!("nop");
}

#[inline(always)]
pub unsafe fn pc_r() -> u32 {
    let r;
    asm!("mov {}, pc", out(reg) r);
    r
}

#[inline(always)]
pub unsafe fn pc_w(val: u32) {
    asm!("mov pc, {}", in(reg) val);
}

#[inline(always)]
pub unsafe fn lr_r() -> u32 {
    let r;
    asm!("mov {}, lr", out(reg) r);
    r
}

#[inline(always)]
pub unsafe fn lr_w(val: u32) {
    asm!("mov lr, {}", in(reg) val);
}

#[inline(always)]
pub unsafe fn primask_r() -> u32 {
    let r;
    asm!("mrs {}, PRIMASK", out(reg) r);
    r
}

#[inline(always)]
pub unsafe fn psp_r() -> u32 {
    let r;
    asm!("mrs {}, PSP", out(reg) r);
    r
}

#[inline(always)]
pub unsafe fn psp_w(val: u32) {
    asm!("msr PSP, {}", in(reg) val);
}

#[inline(always)]
pub unsafe fn sev() {
    asm!("sev");
}

#[inline(always)]
pub unsafe fn udf() -> ! {
    asm!("udf #0", options(noreturn));
}

#[inline(always)]
pub unsafe fn wfe() {
    asm!("wfe");
}

#[inline(always)]
pub unsafe fn wfi() {
    asm!("wfi");
}

/// Semihosting syscall.
#[inline(always)]
pub unsafe fn sh_syscall(mut nr: u32, arg: u32) -> u32 {
    asm!("bkpt #0xab", inout("r0") nr, in("r1") arg);
    nr
}

/// Set CONTROL.SPSEL to 0, write `msp` to MSP, branch to `rv`.
#[inline(always)]
pub unsafe fn bootstrap(msp: u32, rv: u32) -> ! {
    asm!(
        "mrs {tmp}, CONTROL",
        "bics {tmp}, {spsel}",
        "msr CONTROL, {tmp}",
        "isb",
        "msr MSP, {msp}",
        "bx {rv}",
        // `out(reg) _` is not permitted in a `noreturn` asm! call,
        // so instead use `in(reg) 0` and don't restore it afterwards.
        tmp = in(reg) 0,
        spsel = in(reg) 2,
        msp = in(reg) msp,
        rv = in(reg) rv,
        options(noreturn),
    );
}

// v7m *AND* v8m.main, but *NOT* v8m.base
#[cfg(any(armv7m, armv8m_main))]
pub use self::v7m::*;
#[cfg(any(armv7m, armv8m_main))]
mod v7m {
    use core::sync::atomic::{compiler_fence, Ordering};

    #[inline(always)]
    pub unsafe fn basepri_max(val: u8) {
        asm!("msr BASEPRI_MAX, {}", in(reg) val);
    }

    #[inline(always)]
    pub unsafe fn basepri_r() -> u8 {
        let r;
        asm!("mrs {}, BASEPRI", out(reg) r);
        r
    }

    #[inline(always)]
    pub unsafe fn basepri_w(val: u8) {
        asm!("msr BASEPRI, {}", in(reg) val);
    }

    #[inline(always)]
    pub unsafe fn faultmask_r() -> u32 {
        let r;
        asm!("mrs {}, FAULTMASK", out(reg) r);
        r
    }

    #[inline(always)]
    pub unsafe fn enable_icache() {
        asm!(
            "ldr {0}, =0xE000ED14",         // CCR
            "mrs {2}, PRIMASK",             // save critical nesting info
            "cpsid i",                      // mask interrupts
            "ldr {1}, [{0}]",               // read CCR
            "orr.w {1}, {1}, #(1 << 17)",   // Set bit 17, IC
            "str {1}, [{0}]",               // write it back
            "dsb",                          // ensure store completes
            "isb",                          // synchronize pipeline
            "msr PRIMASK, {2}",             // unnest critical section
            out(reg) _,
            out(reg) _,
            out(reg) _,
        );
        compiler_fence(Ordering::SeqCst);
    }

    #[inline(always)]
    pub unsafe fn enable_dcache() {
        asm!(
            "ldr {0}, =0xE000ED14",         // CCR
            "mrs {2}, PRIMASK",             // save critical nesting info
            "cpsid i",                      // mask interrupts
            "ldr {1}, [{0}]",               // read CCR
            "orr.w {1}, {1}, #(1 << 16)",   // Set bit 16, DC
            "str {1}, [{0}]",               // write it back
            "dsb",                          // ensure store completes
            "isb",                          // synchronize pipeline
            "msr PRIMASK, {2}",             // unnest critical section
            out(reg) _,
            out(reg) _,
            out(reg) _,
        );
        compiler_fence(Ordering::SeqCst);
    }
}

#[cfg(armv7em)]
pub use self::v7em::*;
#[cfg(armv7em)]
mod v7em {
    #[inline(always)]
    pub unsafe fn basepri_max_cm7_r0p1(val: u8) {
        asm!(
            "mrs {1}, PRIMASK",
            "cpsid i",
            "tst.w {1}, #1",
            "msr BASEPRI_MAX, {0}",
            "it ne",
            "bxne lr",
            "cpsie i",
            in(reg) val,
            out(reg) _,
        );
    }

    #[inline(always)]
    pub unsafe fn basepri_w_cm7_r0p1(val: u8) {
        asm!(
            "mrs {1}, PRIMASK",
            "cpsid i",
            "tst.w {1}, #1",
            "msr BASEPRI, {0}",
            "it ne",
            "bxne lr",
            "cpsie i",
            in(reg) val,
            out(reg) _,
        );
    }
}

#[cfg(armv8m)]
pub use self::v8m::*;
/// Baseline and Mainline.
#[cfg(armv8m)]
mod v8m {
    #[inline(always)]
    pub unsafe fn tt(mut target: u32) -> u32 {
        asm!("tt {target}, {target}", target = inout(reg) target);
        target
    }

    #[inline(always)]
    pub unsafe fn ttt(mut target: u32) -> u32 {
        asm!("ttt {target}, {target}", target = inout(reg) target);
        target
    }

    #[inline(always)]
    pub unsafe fn tta(mut target: u32) -> u32 {
        asm!("tta {target}, {target}", target = inout(reg) target);
        target
    }

    #[inline(always)]
    pub unsafe fn ttat(mut target: u32) -> u32 {
        asm!("ttat {target}, {target}", target = inout(reg) target);
        target
    }

    #[inline(always)]
    pub unsafe fn msp_ns_r() -> u32 {
        let r;
        asm!("mrs {}, MSP_NS", out(reg) r);
        r
    }

    #[inline(always)]
    pub unsafe fn msp_ns_w(val: u32) {
        asm!("msr MSP_NS, {}", in(reg) val);
    }

    #[inline(always)]
    pub unsafe fn bxns(val: u32) {
        asm!("BXNS {}", in(reg) val);
    }
}

#[cfg(armv8m_main)]
pub use self::v8m_main::*;
/// Mainline only.
#[cfg(armv8m_main)]
mod v8m_main {
    #[inline(always)]
    pub unsafe fn msplim_r() -> u32 {
        let r;
        asm!("mrs {}, MSPLIM", out(reg) r);
        r
    }

    #[inline(always)]
    pub unsafe fn msplim_w(val: u32) {
        asm!("msr MSPLIM, {}", in(reg) val);
    }

    #[inline(always)]
    pub unsafe fn psplim_r() -> u32 {
        let r;
        asm!("mrs {}, PSPLIM", out(reg) r);
        r
    }

    #[inline(always)]
    pub unsafe fn psplim_w(val: u32) {
        asm!("msr PSPLIM, {}", in(reg) val);
    }
}

#[cfg(has_fpu)]
pub use self::fpu::*;

/// All targets with FPU.
#[cfg(has_fpu)]
mod fpu {
    #[inline(always)]
    pub unsafe fn fpscr_r() -> u32 {
        let r;
        asm!("vmrs {}, fpscr", out(reg) r);
        r
    }

    #[inline(always)]
    pub unsafe fn fpscr_w(val: u32) {
        asm!("vmsr fpscr, {}", in(reg) val);
    }
}

}
