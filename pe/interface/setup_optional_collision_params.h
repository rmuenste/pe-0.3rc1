#pragma once

namespace pe {

template <typename CollisionSystemT>
inline auto setOptionalLubrication(CollisionSystemT *cs, bool value)
    -> decltype(cs->setLubrication(value), void()) {
  cs->setLubrication(value);
}

inline void setOptionalLubrication(...) {
  // Active collision system has no lubrication switch.
}

template <typename CollisionSystemT>
inline auto setOptionalSlipLength(CollisionSystemT *cs, real value)
    -> decltype(cs->setSlipLength(value), void()) {
  cs->setSlipLength(value);
}

inline void setOptionalSlipLength(...) {
  // Active collision system has no slip-length parameter.
}

}  // namespace pe
