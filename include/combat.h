#pragma once
#include <stdint.h>

// Physical order: left, front-left, center, front-right, right. Active LOW.
// Kept free of Arduino dependencies so the same policy can be checked on a PC.
struct CombatCommand { int left; int right; const char *action; };

#if __cplusplus >= 201402L
inline constexpr
#else
inline
#endif
CombatCommand combatCommand(uint8_t ir, int left, int right, uint8_t mode, int cap) {
  if (!mode || (!left && !right)) return {left, right, (left || right) ? "VISION" : "HOLD"};
  if (!ir) return {left, right, mode == 2 ? "SEARCH" : "VISION"};
  // Recovery/reverse commands must never be replaced by an IR attack.
  if (mode == 1 && left + right < 0) return {left, right, "VISION"};
  if (cap < 1) cap = 1;
  if (cap > 255) cap = 255;
  const int inner = cap * 65 / 100;
  if ((ir & 4) || ((ir & 2) && (ir & 8))) return {cap, cap, "IR_PUSH"};
  if ((ir & 2) && !(ir & 8)) return {inner, cap, "IR_LEFT"};
  if ((ir & 8) && !(ir & 2)) return {cap, inner, "IR_RIGHT"};
  // Tight rolling turns for flank sensors, rather than an obligatory spot pivot.
  if ((ir & 1) && !(ir & 16)) return {0, cap, "IR_SIDE_LEFT"};
  if ((ir & 16) && !(ir & 1)) return {cap, 0, "IR_SIDE_RIGHT"};
  return {left, right, "IR_AMBIGUOUS"};
}

struct IrFilter {
  uint8_t stable = 0, candidate = 0;
  uint32_t changedAt = 0;
  uint8_t update(uint8_t raw, uint32_t now) {
    if (raw != candidate) { candidate = raw; changedAt = now; }
    if (now - changedAt >= 6) stable = candidate;
    return stable;
  }
};
