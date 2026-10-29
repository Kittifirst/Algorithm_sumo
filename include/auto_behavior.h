#pragma once
#include "combat.h"

#if __cplusplus >= 201402L
#define SUMO_AUTO_CONSTEXPR constexpr
#else
#define SUMO_AUTO_CONSTEXPR inline
#endif

inline constexpr bool useOnboardAuto(uint8_t mode, uint32_t lastDrive, uint32_t ttl,
                                    uint32_t now, bool udpReady) {
  return mode == 2 || uint32_t(now - lastDrive) >= ttl || !udpReady;
}

// Standalone onboard state machine. Search always translates; floor avoidance
// is applied above this policy in main.cpp. No Wi-Fi/camera dependency here.
struct AutoBehavior {
  enum Phase { SEARCH, ATTACK };
  Phase phase = SEARCH;
  uint32_t phaseAt = 0, searchAt = 0, seenAt = 0;
  int bearing = 1;
  uint8_t lastIR = 0;

  SUMO_AUTO_CONSTEXPR void reset(uint32_t now) {
    phase = SEARCH; phaseAt = searchAt = now; seenAt = 0;
    bearing = 1; lastIR = 0;
  }
  SUMO_AUTO_CONSTEXPR void observe(uint8_t ir, uint32_t now) {
    if (!ir) return;
    lastIR = ir; seenAt = now;
    if ((ir & 3) && !(ir & 24)) bearing = 1;
    if ((ir & 24) && !(ir & 3)) bearing = -1;
  }
  SUMO_AUTO_CONSTEXPR void enter(uint32_t now) {
    phase = SEARCH; phaseAt = searchAt = now;
  }
  SUMO_AUTO_CONSTEXPR CombatCommand update(uint8_t ir, uint32_t now, int cap) {
    if (cap < 1) cap = 1;
    if (cap > 255) cap = 255;
    observe(ir, now);
    if (ir) {
      if (phase != ATTACK) { phase = ATTACK; phaseAt = now; }
      // IR alone cannot tell whether the opponent is moving toward a drop.
      // Keep pressure; the camera decides when an actual stalemate needs retreat.
      CombatCommand c = combatCommand(ir, cap, cap, 1, cap);
      return {c.left, c.right, "AUTO_IR_ATTACK"};
    }
    // Bridge a very short IR gap while making physical contact.
    if (phase == ATTACK && lastIR && now - seenAt < 120) {
      CombatCommand c = combatCommand(lastIR, cap, cap, 1, cap);
      return {c.left, c.right, "AUTO_IR_TRACK"};
    }
    if (phase != SEARCH) { phase = SEARCH; searchAt = now; }
    const int fast = cap < 140 ? cap : 140; // Original RUNNING/SEARCH speed.
    const int slow = fast * 80 / 100;
    const uint32_t leg = ((now - searchAt) / 850) % 4;
    if (leg == 0 || leg == 3) return {fast, fast, "AUTO_SEARCH_FWD"};
    const int side = leg == 2 ? -bearing : bearing;
    return {side > 0 ? slow : fast, side > 0 ? fast : slow, "AUTO_SEARCH_ARC"};
  }
};
#undef SUMO_AUTO_CONSTEXPR
