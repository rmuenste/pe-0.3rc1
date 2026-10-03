//=================================================================================================
/*!
 *  \file tools/contact_viewer/StackLab.h
 *  \brief Stack Lab mode of the contact viewer: run / pause / step a small simulation on a ground
 *         plane with the narrow-phase contact overlay and stability diagnostics
 */
//=================================================================================================

#ifndef _PE_TOOLS_CONTACT_VIEWER_STACK_LAB_H_
#define _PE_TOOLS_CONTACT_VIEWER_STACK_LAB_H_

namespace stacklab {

//! Makes Stack Lab the active mode: gravity, visible ground plane, (re)builds the scenario.
//! Requires an initialized Polyscope; call after the other mode's structures were removed.
void activate();

//! Selects scenario \a index (clamped) and rebuilds the world with the staged parameters.
void loadScenario( int index );

//! Advances the simulation by \a steps time steps and refreshes the mirror (headless use).
void advance( int steps );

//! Per-frame callback: simulation controls, stepping, contact overlay, diagnostics plots.
void frame();

//! Headless self check (Polyscope mock backend): every scenario built and stepped for a while,
//! through full GUI frames. Prints stability numbers; returns false on NaNs or exceptions.
bool smokeTest();

} // namespace stacklab

#endif
