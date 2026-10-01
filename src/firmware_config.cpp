#include "firmware_config.h"

namespace firmware {

namespace {
constexpr IncubationProfile PROFILES[] = {
    {"chicken", "Chicken", 37.5F, 58.0F, 70.0F, 21, 18, 2},
    {"duck", "Duck", 37.5F, 56.0F, 70.0F, 28, 25, 2},
    {"muscovy", "Muscovy duck", 37.5F, 56.0F, 70.0F, 35, 32, 2},
    {"coturnix_quail", "Coturnix quail", 37.5F, 56.0F, 70.0F, 17, 14, 2},
    {"pigeon", "Pigeon", 37.5F, 58.0F, 70.0F, 17, 14, 2},
    {"turkey", "Turkey", 37.2F, 56.0F, 70.0F, 28, 25, 2},
    {"goose", "Goose", 37.2F, 62.0F, 70.0F, 30, 27, 2},
    {"guinea_fowl", "Guinea fowl", 37.5F, 58.0F, 70.0F, 28, 25, 2},
};
}

const IncubationProfile *findProfile(const String &id) {
  for (const auto &profile : PROFILES) {
    if (id == profile.id) return &profile;
  }
  return nullptr;
}

}  // namespace firmware
