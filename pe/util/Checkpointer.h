#ifndef _FC2_CHECKPOINTER_HPP_
#define _FC2_CHECKPOINTER_HPP_

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <vector>
#include <list>
#include <boost/numeric/conversion/cast.hpp>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <pe/core/Trigger.h>
#include <pe/core/BodyBinaryWriter.h>
#include <pe/core/BodyBinaryReader.h>

namespace pe {
//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

using boost::filesystem::path;

//*************************************************************************************************
class Checkpointer : public Trigger {
public:
   unsigned int     tspacing_;    //!< Spacing between two visualized time steps.
   unsigned int     tstart_;      //!< First time step to be written
   unsigned int     tend_;        //!< Last time step to be written
   unsigned int     steps_;       //!< Time step counter between two time steps.
   unsigned int     counter_;     //!< Visualization counter for number of visualized time steps.

   explicit Checkpointer()
     : checkpointsPath_(path("checkpoints/")),
       tspacing_(0), tstart_(0), tend_(0), steps_(0), counter_(0)
   {}

   Checkpointer(path checkpointsPath,
                unsigned int spacing,
                unsigned int start,
                unsigned int end)
     : checkpointsPath_(std::move(checkpointsPath)),
       tspacing_(spacing),
       tstart_(start),
       tend_(end),
       steps_(0),
       counter_(0)
   {}

   Checkpointer(const Checkpointer& other)
     : checkpointsPath_(other.checkpointsPath_),
       tspacing_(other.tspacing_),
       tstart_(other.tstart_),
       tend_(other.tend_),
       steps_(other.steps_),
       counter_(other.counter_)
   {}

   ~Checkpointer();

   void trigger();

   /// Write checkpoint under the given name
   void write(const std::string& name);

   /// Read checkpoint under the given name
   void read(const std::string& name);

   /// Wait for any async writes to finish
   void flush() {
      bbwriter_.wait();
   }

   /// Getter for the checkpoints path
   const path& getPath() const {
      return checkpointsPath_;
   }

   /// Setter for the checkpoints path
   void setPath(const path& p) {
      checkpointsPath_ = p;
   }

private:
   BodyBinaryWriter bbwriter_;
   BodyBinaryReader bbreader_;

   /// Directory where checkpoints are saved/loaded
   path             checkpointsPath_;

};
//*************************************************************************************************

} // namespace pe

#endif // _FC2_CHECKPOINTER_HPP_
