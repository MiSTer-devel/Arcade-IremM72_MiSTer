derive_pll_clocks
derive_clock_uncertainty

# core specific constraints

# ===========================================================================
# THE V30 ucore ADVANCES ON A CLOCK ENABLE, NOT ON clk_sys
# ===========================================================================
# Structure follows nec_test's `hdl/nec_test.sdc`.  THE NUMBER IS NOT PORTED --
# it is re-derived for THIS platform's CE train, and it differs from upstream's
# (whose contract C-b buys it two more clocks than m72.v's train gives).
#
# --- the arc, on M72's train -----------------------------------------------
# THE CORE TAKES ONE CLOCK ENABLE (de-muxed build, 2026-08-14).  `CE_HALF` and
# the `t1_half2` flop it gated are gone with the multiplexed AD bus, and with
# them the whole cross-phase question this block used to carry: there is no
# second enable phase, no negedge flop anywhere in the design, and exactly one
# arc left to state.
#
# `rtl/m72.v:183`'s ce_steady train issues `ce_cpu` on the EVEN phase of a
# counter that still advances two per CPU cycle, so two `ce_cpu` pulses are
# never closer than TWO fabric clocks -- 4 in steady state, 2 at the catch-up
# burst rate.  Convention: an enable asserted in cycle k means the flops it
# gates capture at posedge k+1.
#
#   ce -> ce   launch k+1, latch k+3 (burst)   2.0 periods   -setup 2
#
# The odd counter phase is load-bearing FOR THIS NUMBER and m72.v says so at
# the train.  If it is ever collapsed, this becomes `-setup 1`.
#
# --- why the exception is legal at all -------------------------------------
# Both ucore modules are a next-state function (`always @*`, producing `<x>_n`)
# plus a register bank gated `if (ss_we || srst || ce)`, which is the only
# place `ce` appears, so Quartus extracts an `ena` on those flops.  The other
# two enable terms do not break the claim:
#   `srst`   holds for thousands of clocks (ROM load / reset button) and its
#            data is the separately-extracted `<x>_r` / `<x>_rst` reset
#            next-state, so the registers are long settled before the first
#            post-reset `ce` reads the chain.
#   `ss_we`  only pulses while the CE train is stopped (`paused` /
#            `ss_cpu_quiesce`), and m72.v's SST_RESTORE_DRAIN holds CE off for
#            8 further clocks after the last write (rtl/m72.v:556).
#
# FALSIFIER, checkable in the post-fit netlist: a `v30u_eu` or `v30u_biu` state
# register that can take a new value on a clock where `ce` is low.  NOTE the
# `<reg>|ena` probe over-reports -- Quartus folds some enables into a D-side
# feedback mux, which is the same function with no `ena` pin -- so confirm any
# hit by re-checking that register with the exception removed.
set v30u_regs [add_to_collection \
                   [get_registers -nowarn {*|v30u_eu:*|*}] \
                   [get_registers -nowarn {*|v30u_biu:*|*}]]

# The save-state read mux is clocked unconditionally, so it is genuinely
# single-cycle in both directions and is removed for the same reason.
set v30u_ssrd [add_to_collection \
                   [get_registers -nowarn {*|v30u_biu:*|ss_rdata[*]}] \
                   [get_registers -nowarn {*|v30u_eu:*|ss_rdata[*]}]]

set v30u_ce [remove_from_collection $v30u_regs $v30u_ssrd]

if {[get_collection_size $v30u_ce] > 0} {
    set_multicycle_path -setup 2 -from $v30u_ce -to $v30u_ce
    set_multicycle_path -hold  1 -from $v30u_ce -to $v30u_ce
    post_message -type info \
        "Arcade-IremM72.sdc: ucore CE multicycle 2/1 applied to\
         [get_collection_size $v30u_ce] ce-gated v30u registers\
         ([get_collection_size $v30u_ssrd] ss_rdata left at the default check)"
}
