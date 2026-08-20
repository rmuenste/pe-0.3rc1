//=================================================================================================
/*!
 *  \file pe/core/lubrication/ContactState.h
 *  \brief Per-contact lubrication state shared by every stage-capable ContactTrait
 *
 *  Fine detection tags pre-contact pairs through ContactVector::addLubricationContact,
 *  which calls setLubricationFlag() / setLubricationWeight() on the contact. The
 *  lubrication stage (pe/core/lubrication/LubricationStage.h) then reads them back to
 *  decide which contacts it owns and how strongly to blend them.
 *
 *  That storage used to exist on exactly one ContactTrait specialization, the retired
 *  HardContactLubricated one; every other specialization carried no-op stubs whose
 *  getLubricationFlag() returned false. A solver could therefore invoke the stage and
 *  still see zero lubrication contacts -- silently, because the stubs compile fine.
 *
 *  This mixin is the single implementation. A ContactTrait becomes stage-capable by
 *  deriving from it; a solver that genuinely has no use for lubrication keeps using
 *  NullLubricationContactState below, which documents the intent explicitly instead of
 *  looking like an oversight.
 */
//=================================================================================================

#ifndef _PE_CORE_LUBRICATION_CONTACTSTATE_H_
#define _PE_CORE_LUBRICATION_CONTACTSTATE_H_

#include <pe/system/Precision.h>

#include <algorithm>


namespace pe {
namespace lubrication {

//*************************************************************************************************
/*!\brief Real per-contact lubrication state: the tag and its blend weight.
 *
 * The weight is clamped to [0,1] on assignment, so the stage can multiply by it
 * unconditionally.
 */
class ContactState
{
public:
   ContactState()
      : isLubricationContact_( false )
      , lubricationWeight_( real(1) )
   {}

   inline void setLubricationFlag() { isLubricationContact_ = true; }
   inline void setLubricationWeight( real weight ) {
      lubricationWeight_ = std::max( real(0), std::min( real(1), weight ) );
   }
   inline bool getLubricationFlag()   const { return isLubricationContact_; }
   inline real getLubricationWeight() const { return lubricationWeight_; }

private:
   bool isLubricationContact_;
   real lubricationWeight_;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief No-op lubrication state for solvers that deliberately do not run the stage.
 *
 * Satisfies the ContactVector interface while reporting "never a lubrication contact".
 * Costs nothing (empty base) and makes the absence of lubrication an explicit choice.
 */
class NullContactState
{
public:
   inline void setLubricationFlag() {}
   inline void setLubricationWeight( real ) {}
   inline bool getLubricationFlag()   const { return false; }
   inline real getLubricationWeight() const { return real(1); }
};
//*************************************************************************************************

} // namespace lubrication
} // namespace pe

#endif
