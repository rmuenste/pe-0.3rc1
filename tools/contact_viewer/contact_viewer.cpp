//=================================================================================================
/*!
 *  \file tools/contact_viewer/contact_viewer.cpp
 *  \brief Interactive Polyscope viewer for the contact generation and resting contact
 *
 *  Visual debug harness for pe::detection::fine::MaxContacts and the contact solver, two modes
 *  in one binary:
 *   - Stack Lab (stack_lab.cpp, default): run / pause / step a small simulation on a ground
 *     plane (box towers, pyramids, walls, mixed stacks, ramp, drops) with the contact overlay
 *     and stability plots.
 *   - Pair Lab (pair_lab.cpp): pose two bodies and inspect the contacts the narrow phase
 *     generates for them, without time stepping.
 *
 *  Usage: pe_contact_viewer [--pair | --stack] [--smoke]
 *                           [--screenshot <file.png> [--preset <k>] [--steps <n>]]
 *    --pair / --stack   start mode (default: Stack Lab)
 *    --smoke            headless self check of both modes on Polyscope's mock OpenGL backend
 *    --screenshot       render preset/scenario k of the start mode (after n steps in Stack Lab)
 *                       to an image and exit
 */
//=================================================================================================

#include <pe/system/WarningDisable.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <pe/core.h>

#include "polyscope/pick.h"
#include "polyscope/polyscope.h"

#include "implot.h"

#include "PairLab.h"
#include "StackLab.h"

using namespace pe;


namespace {

enum Mode { kStackLab = 0, kPairLab = 1 };
int mode = kStackLab;


void activate( int newMode )
{
   mode = newMode;
   polyscope::pick::resetSelection();
   polyscope::removeAllStructures();
   if( mode == kPairLab )
      pairlab::activate();
   else
      stacklab::activate();
}


void guiCallback()
{
   ImGui::SetNextWindowPos( ImVec2( 10, 10 ), ImGuiCond_FirstUseEver );
   ImGui::Begin( "Mode", nullptr, ImGuiWindowFlags_AlwaysAutoResize );
   int requested = mode;
   ImGui::RadioButton( "Stack Lab (simulation)", &requested, kStackLab );
   ImGui::SameLine();
   ImGui::RadioButton( "Pair Lab (static pair)", &requested, kPairLab );
   ImGui::End();
   if( requested != mode )
      activate( requested );

   if( mode == kPairLab )
      pairlab::frame();
   else
      stacklab::frame();
}

} // namespace


//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

int main( int argc, char** argv )
{
   bool        smoke = false;
   std::string shot;
   int         preset = 0, steps = 0;
   for( int i = 1; i < argc; ++i ) {
      const std::string arg( argv[i] );
      if( arg == "--pair" )                            mode = kPairLab;
      else if( arg == "--stack" )                      mode = kStackLab;
      else if( arg == "--smoke" )                      smoke = true;
      else if( arg == "--screenshot" && i + 1 < argc ) shot = argv[++i];
      else if( arg == "--preset" && i + 1 < argc )     preset = std::atoi( argv[++i] );
      else if( arg == "--steps" && i + 1 < argc )      steps = std::atoi( argv[++i] );
      else {
         std::fprintf( stderr, "usage: %s [--pair | --stack] [--smoke] [--screenshot <file.png> [--preset <k>] [--steps <n>]]\n", argv[0] );
         return EXIT_FAILURE;
      }
   }

   polyscope::options::programName = "PE contact viewer";
   // The callback opens its own windows; skip the empty wrapper window Polyscope
   // would otherwise create for it.
   polyscope::options::openImGuiWindowForUserCallback = false;
   polyscope::options::transparencyMode = polyscope::TransparencyMode::Pretty;
   polyscope::view::setUpDir( polyscope::UpDir::ZUp );
   polyscope::init( smoke ? "openGL_mock" : "" );
   ImPlot::CreateContext();

   activate( mode );
   polyscope::state::userCallback = guiCallback;

   bool ok = true;
   if( smoke ) {
      activate( kPairLab );
      ok = pairlab::smokeTest();
      activate( kStackLab );
      ok = stacklab::smokeTest() && ok;
      std::printf( "\nsmoke test %s\n", ok ? "passed" : "FAILED" );
   }
   else if( !shot.empty() ) {
      if( mode == kPairLab ) {
         pairlab::loadPreset( preset );
      }
      else {
         stacklab::loadScenario( preset );
         stacklab::advance( steps );
      }
      for( int i = 0; i < 3; ++i )   // the first frames settle the overlay and the camera
         polyscope::frameTick();
      polyscope::screenshot( shot, /*transparentBG=*/false );
   }
   else {
      polyscope::show();
   }

   ImPlot::DestroyContext();
   return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
