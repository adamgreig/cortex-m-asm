// Many of these intrinsics are copied from
// https://github.com/rust-embedded/cortex-m/blob/master/asm/inline.rs
// Please refer to that repository for original authorship.

#![no_std]
#![cfg_attr(not(feature = "do-crimes"), feature(asm))]
#![allow(unused_unsafe)]

#[cfg(feature = "do-crimes")]
use nightly_crimes::nightly_crimes;

#[cfg(not(feature = "do-crimes"))]
#[allow(unused_macros)]
macro_rules! nightly_crimes {
    (#![feature(asm)] $($code:tt)*) => { $($code)* }
}

#[cfg(cortex_m)]
nightly_crimes! {

#![feature(asm)]

use core::sync::atomic::{compiler_fence, Ordering};

/// Puts the processor in Debug state. Debuggers can pick this up as a "breakpoint".
///
/// **NOTE** calling `bkpt` when the processor is not connected to a debugger will cause an
/// exception.
#[inline(always)]
pub fn bkpt() {
    unsafe { asm!("bkpt") };
}

/// Reads the CONTROL register.
#[inline(always)]
pub fn control_r() -> u32 {
    let r;
    unsafe { asm!("mrs {}, CONTROL", out(reg) r) };
    r
}

/// Writes the CONTROL register.
#[inline(always)]
pub unsafe fn control_w(w: u32) {
    // ISB is required after writing to CONTROL,
    // per ARM architectural requirements (see Application Note 321).
    unsafe { asm!(
        "msr CONTROL, {}",
        "isb",
        in(reg) w
    )};

    // Ensure memory accesses are not reordered around the CONTROL update.
    compiler_fence(Ordering::SeqCst);
}

/// Disables all interrupts.
#[inline(always)]
pub fn cpsid() {
    unsafe { asm!("cpsid i") };

    // Ensure no subsequent memory accesses are reordered to before interrupts are disabled.
    compiler_fence(Ordering::SeqCst);
}

/// Enables all interrupts.
///
/// # Safety
///
/// - Do not call this function inside an `interrupt::free` critical section.
#[inline(always)]
pub unsafe fn cpsie() {
    // Ensure no preceeding memory accesses are reordered to after interrupts are enabled.
    compiler_fence(Ordering::SeqCst);

    unsafe { asm!("cpsie i") };
}

/// Blocks the program for *at least* `cycles` CPU cycles.
///
/// This is implemented in assembly so its execution time is independent of the optimization
/// level, however it is dependent on the specific architecture and core configuration.
///
/// NOTE that the delay can take much longer if interrupts are serviced during its execution
/// and the execution time may vary with other factors. This delay is mainly useful for simple
/// timer-less initialization of peripherals if and only if accurate timing is not essential. In
/// any other case please use a more accurate method to produce a delay.
#[inline]
pub fn delay(cyc: u32) {
    // The loop will normally take 3 to 4 CPU cycles per iteration, but superscalar cores
    // (eg. Cortex-M7) can potentially do it in 2, so we use that as the lower bound, since delaying
    // for more cycles is okay.
    // Add 1 to prevent an integer underflow which would cause a long freeze
    let real_cyc = 1 + cyc / 2;
    unsafe { asm!(
        // Use local labels to avoid R_ARM_THM_JUMP8 relocations which fail on thumbv6m.
        "1:",
        "subs {}, #1",
        "bne 1b",
        inout(reg) real_cyc => _
    )};
}

/// Data Memory Barrier
///
/// Ensures that all explicit memory accesses that appear in program order before the `DMB`
/// instruction are observed before any explicit memory accesses that appear in program order
/// after the `DMB` instruction.
#[inline(always)]
pub fn dmb() {
    compiler_fence(Ordering::SeqCst);
    unsafe { asm!("dmb") };
    compiler_fence(Ordering::SeqCst);
}

/// Data Synchronization Barrier
///
/// Acts as a special kind of memory barrier. No instruction in program order after this instruction
/// can execute until this instruction completes. This instruction completes only when both:
///
///  * any explicit memory access made before this instruction is complete
///  * all cache and branch predictor maintenance operations before this instruction complete
#[inline(always)]
pub fn dsb() {
    compiler_fence(Ordering::SeqCst);
    unsafe { asm!("dsb") };
    compiler_fence(Ordering::SeqCst);
}

/// Instruction Synchronization Barrier
///
/// Flushes the pipeline in the processor, so that all instructions following the `ISB` are fetched
/// from cache or memory, after the instruction has been completed.
#[inline(always)]
pub fn isb() {
    compiler_fence(Ordering::SeqCst);
    unsafe { asm!("isb") };
    compiler_fence(Ordering::SeqCst);
}

/// Read the MSP register.
#[inline(always)]
pub fn msp_r() -> u32 {
    let r;
    unsafe { asm!("mrs {}, MSP", out(reg) r) };
    r
}

/// Write the MSP register.
#[inline(always)]
pub unsafe fn msp_w(val: u32) {
    unsafe { asm!("msr MSP, {}", in(reg) val) };
}

/// Read the APSR register.
#[inline(always)]
pub fn apsr_r() -> u32 {
    let r;
    unsafe { asm!("mrs {}, APSR", out(reg) r) };
    r
}

/// A no-operation. Useful to prevent delay loops from being optimized away.
#[inline(always)]
pub fn nop() {
    // NOTE: This is a `pure` asm block, but applying that option allows the compiler to eliminate
    // the nop entirely (or to collapse multiple subsequent ones). Since the user probably wants N
    // nops when they call `nop` N times, let's not add that option.
    unsafe { asm!("nop") };
}

/// Read the PC register.
#[inline(always)]
pub fn pc_r() -> u32 {
    let r;
    unsafe { asm!("mov {}, pc", out(reg) r) };
    r
}

/// Write to the PC register.
#[inline(always)]
pub unsafe fn pc_w(val: u32) {
    unsafe { asm!("mov pc, {}", in(reg) val) };
}

/// Read the LR register.
#[inline(always)]
pub fn lr_r() -> u32 {
    let r;
    unsafe { asm!("mov {}, lr", out(reg) r) };
    r
}

/// Write to the LR register.
#[inline(always)]
pub unsafe fn lr_w(val: u32) {
    unsafe { asm!("mov lr, {}", in(reg) val) };
}

/// Read the PRIMASK register.
#[inline(always)]
pub fn primask_r() -> u32 {
    let r;
    unsafe { asm!("mrs {}, PRIMASK", out(reg) r) };
    r
}

/// Read the PSP register.
#[inline(always)]
pub fn psp_r() -> u32 {
    let r;
    unsafe { asm!("mrs {}, PSP", out(reg) r) };
    r
}

/// Write the PSP register.
#[inline(always)]
pub unsafe fn psp_w(val: u32) {
    unsafe { asm!("msr PSP, {}", in(reg) val) };
}

/// Send Event.
#[inline(always)]
pub fn sev() {
    unsafe { asm!("sev") };
}

/// Generate an Undefined Instruction exception.
///
/// Can be used as a stable alternative to `core::intrinsics::abort`.
#[inline(always)]
pub fn udf() -> ! {
    unsafe { asm!("udf #0", options(noreturn)) };
}

/// Wait For Event.
#[inline(always)]
pub fn wfe() {
    unsafe { asm!("wfe") };
}

/// Wait For Interrupt.
#[inline(always)]
pub fn wfi() {
    unsafe { asm!("wfi") };
}

/// Semihosting syscall.
#[inline(always)]
pub unsafe fn sh_syscall(mut nr: u32, arg: u32) -> u32 {
    unsafe { asm!("bkpt #0xab", inout("r0") nr, in("r1") arg) };
    nr
}

/// Set CONTROL.SPSEL to 0, write `msp` to MSP, branch to `rv`.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[inline(always)]
pub unsafe fn bootstrap(msp: *const u32, rv: *const u32) -> ! {
    let msp = msp as u32;
    let rv = rv as u32;
    unsafe { asm!(
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
    )};
}

// v7m *AND* v8m.main, but *NOT* v8m.base
#[cfg(any(armv7m, armv8m_main))]
pub use self::v7m::*;
#[cfg(any(armv7m, armv8m_main))]
mod v7m {
    use core::sync::atomic::{compiler_fence, Ordering};

    /// Write the BASEPRI_MAX register.
    #[inline(always)]
    pub unsafe fn basepri_max(val: u8) {
        unsafe { asm!("msr BASEPRI_MAX, {}", in(reg) val) };
    }

    /// Read the BASEPRI register.
    #[inline(always)]
    pub fn basepri_r() -> u8 {
        let r;
        unsafe { asm!("mrs {}, BASEPRI", out(reg) r) };
        r
    }

    /// Write the BASEPRI register.
    #[inline(always)]
    pub unsafe fn basepri_w(val: u8) {
        unsafe { asm!("msr BASEPRI, {}", in(reg) val) };
    }

    /// Read the FAULTMASK register.
    #[inline(always)]
    pub fn faultmask_r() -> u32 {
        let r;
        unsafe { asm!("mrs {}, FAULTMASK", out(reg) r) };
        r
    }

    /// Enable ICACHE.
    ///
    /// The ICACHE must be invalidated before enabling.
    ///
    /// This method manages exclusive access to the SCB registers using a critical section.
    #[inline(always)]
    pub unsafe fn enable_icache() {
        unsafe {asm!(
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
        )};
        compiler_fence(Ordering::SeqCst);
    }

    /// Enable DCACHE.
    ///
    /// The DCACHE must be invalidated before enabling.
    ///
    /// This method manages exclusive access to the SCB registers using a critical section.
    #[inline(always)]
    pub unsafe fn enable_dcache() {
        unsafe { asm!(
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
        )};
        compiler_fence(Ordering::SeqCst);
    }
}

#[cfg(armv7em)]
pub use self::v7em::*;
#[cfg(armv7em)]
mod v7em {
    /// Write to BASEPRI_MAX on Cortex-M7 r0p1 CPUs.
    /// Accounts for erratum 837070.
    #[inline(always)]
    pub unsafe fn basepri_max_cm7_r0p1(val: u8) {
        unsafe { asm!(
            "mrs {1}, PRIMASK",
            "cpsid i",
            "tst.w {1}, #1",
            "msr BASEPRI_MAX, {0}",
            "it ne",
            "bxne lr",
            "cpsie i",
            in(reg) val,
            out(reg) _,
        )};
    }

    /// Write to BASEPRI on Cortex-M7 r0p1 CPUs.
    /// Accounts for erratum 837070.
    #[inline(always)]
    pub unsafe fn basepri_w_cm7_r0p1(val: u8) {
        unsafe { asm!(
            "mrs {1}, PRIMASK",
            "cpsid i",
            "tst.w {1}, #1",
            "msr BASEPRI, {0}",
            "it ne",
            "bxne lr",
            "cpsie i",
            in(reg) val,
            out(reg) _,
        )};
    }
}

#[cfg(armv8m)]
pub use self::v8m::*;
/// Baseline and Mainline.
#[cfg(armv8m)]
mod v8m {
    /// Test Target
    ///
    /// Queries the Security state and access permissions of a memory location.
    ///
    /// Returns a Test Target Response Payload (cf section D1.2.215 of
    /// Armv8-M Architecture Reference Manual).
    #[allow(clippy::not_unsafe_ptr_arg_deref)]
    #[inline(always)]
    pub fn tt(mut target: *mut u32) -> u32 {
        let mut target = target as u32;
        unsafe { asm!("tt {target}, {target}", target = inout(reg) target) };
        target
    }

    /// Test Target Unprivileged
    ///
    /// Queries the Security state and access permissions of a memory location for an unprivileged
    /// access to that location.
    ///
    /// Returns a Test Target Response Payload (cf section D1.2.215 of
    /// Armv8-M Architecture Reference Manual).
    #[allow(clippy::not_unsafe_ptr_arg_deref)]
    #[inline(always)]
    pub fn ttt(mut target: *mut u32) -> u32 {
        let mut target = target as u32;
        unsafe { asm!("ttt {target}, {target}", target = inout(reg) target) };
        target
    }

    /// Test Target Alternate Domain
    ///
    /// Queries the Security state and access permissions of a memory location for a Non-Secure
    /// access to that location. This instruction is only valid when executing in Secure state and
    /// is undefined if used from Non-Secure state.
    ///
    /// Returns a Test Target Response Payload (cf section D1.2.215 of
    /// Armv8-M Architecture Reference Manual).
    #[allow(clippy::not_unsafe_ptr_arg_deref)]
    #[inline(always)]
    pub fn tta(mut target: *mut u32) -> u32 {
        let mut target = target as u32;
        unsafe { asm!("tta {target}, {target}", target = inout(reg) target) };
        target
    }

    /// Test Target Alternate Domain Unprivileged
    ///
    /// Queries the Security state and access permissions of a memory location for a Non-Secure and
    /// unprivileged access to that location. This instruction is only valid when executing in
    /// Secure state and is undefined if used from Non-Secure state.
    ///
    /// Returns a Test Target Response Payload (cf section D1.2.215 of
    /// Armv8-M Architecture Reference Manual).
    #[allow(clippy::not_unsafe_ptr_arg_deref)]
    #[inline(always)]
    pub fn ttat(mut target: *mut u32) -> u32 {
        let mut target = target as u32;
        unsafe { asm!("ttat {target}, {target}", target = inout(reg) target) };
        target
    }

    /// Reads the Non-Secure MSP register from the Secure state.
    ///
    /// Executing this function in Non-Secure state will return 0.
    #[inline(always)]
    pub fn msp_ns_r() -> u32 {
        let r;
        unsafe { asm!("mrs {}, MSP_NS", out(reg) r) };
        r
    }

    /// Writes to the Non-Secure MSP register from the Secure state.
    ///
    /// Executing this function in Non-Seure state will be ignored.
    #[inline(always)]
    pub unsafe fn msp_ns_w(val: u32) {
        unsafe { asm!("msr MSP_NS, {}", in(reg) val) };
    }

    /// Branch and Exchange Non-secure
    ///
    /// See section C2.4.26 of Armv8-M Architecture Reference Manual for details.
    /// Undefined if executed in Non-Secure state.
    #[inline(always)]
    pub unsafe fn bxns(val: u32) {
        unsafe { asm!("BXNS {}", in(reg) val) };
    }
}

#[cfg(armv8m_main)]
pub use self::v8m_main::*;
/// Mainline only.
#[cfg(armv8m_main)]
mod v8m_main {
    /// Reads the MSPLIM register.
    #[inline(always)]
    pub fn msplim_r() -> u32 {
        let r;
        unsafe { asm!("mrs {}, MSPLIM", out(reg) r) };
        r
    }

    /// Writes to the MSPLIM register.
    #[inline(always)]
    pub unsafe fn msplim_w(val: u32) {
        unsafe { asm!("msr MSPLIM, {}", in(reg) val) };
    }

    /// Reads the PSPLIM register.
    #[inline(always)]
    pub fn psplim_r() -> u32 {
        let r;
        unsafe { asm!("mrs {}, PSPLIM", out(reg) r) };
        r
    }

    /// Writes to the PSPLIM register.
    #[inline(always)]
    pub unsafe fn psplim_w(val: u32) {
        unsafe { asm!("msr PSPLIM, {}", in(reg) val) };
    }
}

#[cfg(has_fpu)]
pub use self::fpu::*;

/// All targets with FPU.
#[cfg(has_fpu)]
mod fpu {
    /// Reads the FPSCR register.
    #[inline(always)]
    pub fn fpscr_r() -> u32 {
        let r;
        unsafe { asm!("vmrs {}, fpscr", out(reg) r) };
        r
    }

    /// Writes to the FPSCR register.
    #[inline(always)]
    pub unsafe fn fpscr_w(val: u32) {
        unsafe { asm!("vmsr fpscr, {}", in(reg) val) };
    }
}

}
