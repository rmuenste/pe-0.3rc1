#pragma once

//=================================================================================================
// Start-up echo of the effective contact parameters of the EL setups (one greppable line):
//
//   EL_CONTACT_PARAMS pair=particle-particle mu_static=0.3 mu_dynamic=0.3 e=0.9
//     | pair=particle-wall wall=zwalls mu_static=0.3 mu_dynamic=0.3 e=0.9
//     | restitution=on threshold=0 scheme=post-pass iterations=4 max_iterations=24 check_interval=4 abs_tolerance=0.0001 rel_tolerance=0.001
//
// (printed as a single line). The pair values are the ones a contact uses: pe COMBINES the
// material values (Material::getStaticFriction( m1, m2 ) = csf1 + csf2, the restitution a
// stiffness-weighted mean, see Material::createMaterial), so json staticFriction_ 0.15 shows as
// mu_static=0.3. The tail is the state of the collision system's restitution switch after
// applyOptionalRestitutionParams(): with restitution=off every contact is inelastic and e= is
// the material pair value that a bounce would use. threshold= is the user threshold; the solver
// adds its resting bound (HardContactEulerLagrange::setRestitutionThreshold()).
//=================================================================================================

#include <pe/core/Materials.h>
#include <pe/interface/support/optional_collision_params.h>

#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace pe {

//! Wall kinds of the echo: (name, material) of every fixed wall the setup created.
typedef std::vector<std::pair<std::string, MaterialID>> ContactEchoWalls;

//*************************************************************************************************
/*!\brief The " mu_static=.. mu_dynamic=.. e=.." part of one pair. */
inline std::string contactPairEchoValues(MaterialID m1, MaterialID m2) {
  std::ostringstream os;
  os.precision(10);
  os << " mu_static=" << Material::getStaticFriction(m1, m2)
     << " mu_dynamic=" << Material::getDynamicFriction(m1, m2)
     << " e=" << Material::getRestitution(m1, m2);
  return os.str();
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief The " restitution=.. threshold=.. scheme=.. iterations=.." tail (state of the solver). */
template <typename CollisionSystemT>
inline std::string contactRestitutionEchoValues(const CollisionSystemT& cs) {
  std::ostringstream os;
  os.precision(10);
  if constexpr (HasRestitutionSwitch<CollisionSystemT>::value) {
    os << " restitution=" << (cs.getRestitution() ? "on" : "off")
       << " threshold=" << cs.getRestitutionThreshold()
       << " scheme=" << (cs.getRestitutionPostPass() ? "post-pass" : "in-solve")
       << " iterations=" << cs.getRestitutionIterations()
       << " max_iterations=" << cs.getRestitutionMaxIterations()
       << " check_interval=" << cs.getRestitutionCheckInterval()
       << " abs_tolerance=" << cs.getRestitutionAbsTolerance()
       << " rel_tolerance=" << cs.getRestitutionRelTolerance();
  } else {
    (void)cs;
    os << " restitution=unsupported";
  }
  return os.str();
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief The EL_CONTACT_PARAMS line (without newline).
 *
 * \param cs The collision system (after applyOptionalRestitutionParams()).
 * \param particle The material of the free particles.
 * \param walls The fixed walls of the setup; empty gives "pair=particle-wall wall=none".
 */
template <typename CollisionSystemT>
inline std::string elContactParamsLine(const CollisionSystemT& cs, MaterialID particle,
                                       const ContactEchoWalls& walls) {
  std::string line("EL_CONTACT_PARAMS pair=particle-particle");
  line += contactPairEchoValues(particle, particle);
  if (walls.empty()) {
    line += " | pair=particle-wall wall=none";
  }
  for (const auto& w : walls) {
    line += " | pair=particle-wall wall=" + w.first + contactPairEchoValues(particle, w.second);
  }
  line += " |" + contactRestitutionEchoValues(cs);
  return line;
}
//*************************************************************************************************

}  // namespace pe
