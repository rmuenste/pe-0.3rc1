//=================================================================================================
/*!
 *  \file tools/contact_viewer/PairLab.h
 *  \brief Pair Lab mode of the contact viewer: static narrow-phase inspection of two bodies
 */
//=================================================================================================

#ifndef _PE_TOOLS_CONTACT_VIEWER_PAIR_LAB_H_
#define _PE_TOOLS_CONTACT_VIEWER_PAIR_LAB_H_

namespace pairlab {

//! Makes Pair Lab the active mode: no gravity, no ground, (re)builds the posed pair. Requires
//! an initialized Polyscope; call after the other mode's structures were removed.
void activate();

//! Loads the preset pair \a index (clamped to the preset list).
void loadPreset( int index );

//! Per-frame callback: pose controls, contact generation, overlay, table, sweep plots.
void frame();

//! Headless self check (Polyscope mock backend): every preset through a full frame plus the
//! pose round trip of the gizmo read-back. Prints the contacts; returns false on failure.
bool smokeTest();

} // namespace pairlab

#endif
