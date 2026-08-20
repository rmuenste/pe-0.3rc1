//=================================================================================================
/*!
 *  \file tools/live_viewer/ParamRegistry.h
 *  \brief Tiny declarative parameter registry for the live viewer GUI
 *
 *  Each engine knob is described once as a {name, range, getter, setter} entry; the GUI
 *  iterates the registry and renders a widget per entry. Adding a new tweakable is one
 *  push_back. Getters pull the authoritative value from the engine every frame, so values
 *  changed elsewhere (config files, interface layer) stay visible in the GUI; setters push
 *  edits back immediately. For engine knobs without a getter, capture a shadow variable in
 *  both lambdas.
 */
//=================================================================================================

#ifndef _PE_TOOLS_LIVE_VIEWER_PARAM_REGISTRY_H_
#define _PE_TOOLS_LIVE_VIEWER_PARAM_REGISTRY_H_

#include <functional>
#include <string>
#include <vector>

#include "imgui.h"

namespace viewer {

struct RealParam {
   std::string name;
   float       min;
   float       max;
   bool        logScale;
   std::function<double()>     get;
   std::function<void(double)> set;
   const char* tooltip = nullptr;
};

struct BoolParam {
   std::string name;
   std::function<bool()>     get;
   std::function<void(bool)> set;
   const char* tooltip = nullptr;
};

//! Combo-box entry for enum-like int knobs (e.g. lubrication model/scheme switches).
struct EnumParam {
   std::string name;
   std::vector<const char*> items;
   std::function<int()>     get;
   std::function<void(int)> set;
   const char* tooltip = nullptr;
};

struct ParamGroup {
   std::string name;
   std::vector<BoolParam> bools;
   std::vector<EnumParam> enums;
   std::vector<RealParam> reals;
};

inline void showTooltip( const char* text )
{
   if( text != nullptr && ImGui::IsItemHovered( ImGuiHoveredFlags_DelayShort ) )
      ImGui::SetTooltip( "%s", text );
}

inline void drawParamGroup( ParamGroup& group )
{
   if( !ImGui::CollapsingHeader( group.name.c_str(), ImGuiTreeNodeFlags_DefaultOpen ) )
      return;

   ImGui::PushID( group.name.c_str() );

   for( auto& p : group.bools ) {
      bool value = p.get();
      if( ImGui::Checkbox( p.name.c_str(), &value ) )
         p.set( value );
      showTooltip( p.tooltip );
   }

   for( auto& p : group.enums ) {
      int value = p.get();
      if( ImGui::Combo( p.name.c_str(), &value, p.items.data(), static_cast<int>( p.items.size() ) ) )
         p.set( value );
      showTooltip( p.tooltip );
   }

   for( auto& p : group.reals ) {
      float value = static_cast<float>( p.get() );
      const ImGuiSliderFlags flags = p.logScale ? ImGuiSliderFlags_Logarithmic : ImGuiSliderFlags_None;
      if( ImGui::SliderFloat( p.name.c_str(), &value, p.min, p.max, "%.6g", flags ) )
         p.set( static_cast<double>( value ) );
      showTooltip( p.tooltip );
   }

   ImGui::PopID();
}

inline void drawParamGroups( std::vector<ParamGroup>& groups )
{
   for( auto& group : groups )
      drawParamGroup( group );
}

} // namespace viewer

#endif
