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

   unsigned int tspacing_;       //!< Spacing between two visualized time steps.
   unsigned int tstart_;         //!< First time step to be written
   unsigned int tend_;           //!< last time step to be written
   unsigned int steps_;          //!< Time step counter between two time steps.
   unsigned int counter_;        //!< Visualization counter for the number of visualized time steps.

   explicit Checkpointer() :
     checkpointsPath_( path( "checkpoints/" ) ),
     tspacing_(0), tstart_(0), tend_(0), steps_(0), counter_(0)
   {}

   Checkpointer(path checkpointsPath, unsigned int spacing, unsigned int start, unsigned int end) :
     checkpointsPath_( path( "checkpoints/" ) ),
     tspacing_(spacing), tstart_(start), tend_(end), steps_(0), counter_(0)
   {}
   
   Checkpointer( const Checkpointer& other ) : 
     checkpointsPath_( other.checkpointsPath_ ),
     tspacing_(other.tspacing_), tstart_(other.tstart_), 
     tend_(other.tend_), steps_(other.steps_), counter_(other.counter_)
   {
   }

   ~Checkpointer();  

   void trigger();

   void setPath( path checkpointsPath = path( "checkpoints/" ) ) {
      checkpointsPath_ = checkpointsPath;
   }

   void write(std::string name); 

   void read(std::string name);

   void flush() {
      bbwriter_.wait();
   }

private:

   BodyBinaryWriter bbwriter_;
   BodyBinaryReader bbreader_;
   path             checkpointsPath_;
};
//*************************************************************************************************

}

#endif
