// Compile-time checks execute the actual firmware IR policy, without motors.
// xtensa-esp32-elf-g++ -std=c++14 -Iinclude -fsyntax-only tools/verify_combat.cpp
#include "combat.h"
#include "auto_behavior.h"
#include <initializer_list>

static_assert(combatCommand(4, 100, 100, 1, 130).left == 130, "center IR must push");
static_assert(combatCommand(4, 100, 100, 1, 130).right == 130, "center IR must push both wheels");
static_assert(combatCommand(2, 100, 100, 1, 130).left < combatCommand(2, 100, 100, 1, 130).right,
              "front-left IR must curve left");
static_assert(combatCommand(8, 100, 100, 1, 130).left > combatCommand(8, 100, 100, 1, 130).right,
              "front-right IR must curve right");
static_assert(combatCommand(1, 100, 100, 1, 130).left == 0, "left-side capture must roll around inner wheel");
static_assert(combatCommand(16, 100, 100, 1, 130).right == 0, "right-side capture must roll around inner wheel");
static_assert(combatCommand(31, -90, -90, 1, 150).left == -90, "IR must not defeat retreat");
static_assert(combatCommand(4, 0, 0, 0, 150).left == 0, "raw vision hold/stop must not be overridden");
static_assert(combatCommand(4, -70, 70, 2, 255).left == 255, "IR acquisition during search must use full configured power");
static_assert(combatCommand(4, 255, 255, 1, 255).right == 255, "full-power center push");
static_assert(combatCommand(31, -255, -255, 1, 255).left == -255, "full-power retreat must remain a retreat");
static_assert(combatCommand(4, -70, 70, 1, 130).left == 130, "IR must capture during a vision pivot too");
static_assert(combatCommand(4, 100, 100, 0, 150).left == 100, "mode 0 must disable IR override");

constexpr bool all_masks_obey_cap() {
  for (int mask = 0; mask < 32; ++mask) {
    for (int cap : {60, 90, 130, 150, 255}) {
      CombatCommand c = combatCommand(mask, 60, 60, 1, cap);
      if (c.left < 0 || c.right < 0 || c.left > cap || c.right > cap) return false;
    }
  }
  return true;
}
static_assert(all_masks_obey_cap(), "all IR combinations must preserve configured motor limits");

static_assert(useOnboardAuto(1, 100, 350, 451, true), "expired vision lease must select onboard AUTO");
static_assert(useOnboardAuto(1, 100, 350, 120, false), "Wi-Fi loss must select onboard AUTO");
static_assert(useOnboardAuto(2, 100, 350, 120, true), "explicit fallback must run onboard AUTO even with zero host wheels");
static_assert(!useOnboardAuto(1, 100, 350, 120, true), "fresh camera commands must regain control");

constexpr bool walking_search_and_ir_capture() {
  AutoBehavior a{};
  a.reset(0);
  for (uint32_t now = 0; now < 15000; now += 100) {
    CombatCommand c = a.update(0, now, 255);
    if (c.left <= 0 || c.right <= 0 || c.left > 140 || c.right > 140) return false;
  }
  CombatCommand hit = a.update(4, 15000, 255);
  return hit.left == 255 && hit.right == 255;
}
static_assert(walking_search_and_ir_capture(), "search must keep walking, and IR must immediately take over");

constexpr bool sustained_ir_pressure() {
  AutoBehavior a{};
  a.reset(0);
  for (uint32_t now = 1; now < 30000; now += 100) {
    CombatCommand c = a.update(4, now, 255);
    if (c.left != 255 || c.right != 255) return false;
  }
  return true;
}
static_assert(sustained_ir_pressure(), "IR pressure must not retreat on a timer; camera confirms real stalls");
