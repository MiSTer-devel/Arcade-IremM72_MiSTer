#ifndef SIM_HIERARCHY_H
#define SIM_HIERARCHY_H

// Variadic macro to simplify accessing M72 core signals through the sim_top
// wrapper.
// Usage: M72_SIGNAL(sound, sndram, ram) -> sim_top__DOT__m72_inst__DOT__sound__DOT__sndram__DOT__ram
// Usage: M72_SIGNAL(paused)            -> sim_top__DOT__m72_inst__DOT__paused

#define M72_SIGNAL_1(a) sim_top__DOT__m72_inst__DOT__##a
#define M72_SIGNAL_2(a, b) sim_top__DOT__m72_inst__DOT__##a##__DOT__##b
#define M72_SIGNAL_3(a, b, c) sim_top__DOT__m72_inst__DOT__##a##__DOT__##b##__DOT__##c
#define M72_SIGNAL_4(a, b, c, d) sim_top__DOT__m72_inst__DOT__##a##__DOT__##b##__DOT__##c##__DOT__##d
#define M72_SIGNAL_5(a, b, c, d, e) sim_top__DOT__m72_inst__DOT__##a##__DOT__##b##__DOT__##c##__DOT__##d##__DOT__##e

// Count arguments macro
#define _M72_GET_ARG_COUNT(...) _M72_GET_ARG_COUNT_IMPL(__VA_ARGS__, 5, 4, 3, 2, 1, 0)
#define _M72_GET_ARG_COUNT_IMPL(_1, _2, _3, _4, _5, N, ...) N

// Main macro that dispatches to the correct arity version
#define M72_SIGNAL(...) _M72_SIGNAL_DISPATCH(_M72_GET_ARG_COUNT(__VA_ARGS__), __VA_ARGS__)
#define _M72_SIGNAL_DISPATCH(N, ...) _M72_SIGNAL_CONCAT(M72_SIGNAL_, N)(__VA_ARGS__)
#define _M72_SIGNAL_CONCAT(a, b) a##b

#define G_M72_SIGNAL(...) gSimCore.mTop->rootp->_M72_SIGNAL_DISPATCH(_M72_GET_ARG_COUNT(__VA_ARGS__), __VA_ARGS__)

#endif // SIM_HIERARCHY_H
