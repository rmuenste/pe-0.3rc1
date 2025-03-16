#if HAVE_MPI
#include <pe/core/MPI.h>
#include <pe/config/Precision.h>
#include <boost/filesystem.hpp>

namespace pe {

/**
 * @brief Simulation configuration class to encapsulate all simulation parameters
 */
class SimulationConfig {
public:
    // Singleton accessor
    static SimulationConfig& getInstance();

    // Time parameters
    size_t getTimesteps() const { return timesteps_; }
    real getStepsize() const { return stepsize_; }
    void setTimesteps(size_t value) { timesteps_ = value; }
    void setStepsize(real value) { stepsize_ = value; }

    // Process parameters
    int getProcessesX() const { return processesX_; }
    int getProcessesY() const { return processesY_; }
    int getProcessesZ() const { return processesZ_; }
    void setProcessesX(int value) { processesX_ = value; }
    void setProcessesY(int value) { processesY_ = value; }
    void setProcessesZ(int value) { processesZ_ = value; }

    // Random number generator parameters
    size_t getSeed() const { return seed_; }
    void setSeed(size_t value) { seed_ = value; }

    // Verbose mode
    bool getVerbose() const { return verbose_; }
    void setVerbose(bool value) { verbose_ = value; }

    // VTK visualization flag
    bool getVtk() const { return vtk_; }
    void setVtk(bool value) { vtk_ = value; }

    // Visualization parameters
    unsigned int getVisspacing() const { return visspacing_; }
    unsigned int getPointerspacing() const { return pointerspacing_; }
    void setVisspacing(unsigned int value) { visspacing_ = value; }
    void setPointerspacing(unsigned int value) { pointerspacing_ = value; }

    // Checkpointer usage
    bool getUseCheckpointer() const { return useCheckpointer_; }
    void setUseCheckpointer(bool value) { useCheckpointer_ = value; }

    // Process convenience accessors
    int getPx() const { return processesX_; }
    int getPy() const { return processesY_; }
    int getPz() const { return processesZ_; }

    // Checkpoint path
    const boost::filesystem::path& getCheckpointPath() const { return checkpoint_path_; }
    void setCheckpointPath(const boost::filesystem::path& path) { checkpoint_path_ = path; }

private:
    // Private constructor for singleton
    SimulationConfig();
    
    // Time parameters
    size_t timesteps_ = 15000;    // Number of time steps for the flowing granular media
    real stepsize_ = 0.001;       // Size of a single time step
    
    // Process parameters
    int processesX_ = 3;          // Number of processes in x-direction
    int processesY_ = 3;          // Number of processes in y-direction
    int processesZ_ = 3;          // Number of processes in z-direction
    
    // Random number generator parameters
    size_t seed_ = 12345;
    
    // Verbose mode
    bool verbose_ = false;        // Switches the output of the simulation on and off
    
    // VTK visualization flag
    bool vtk_ = true;
    
    // Visualization parameters
    unsigned int visspacing_ = 50;       // Spacing between two visualizations (POV-Ray & Irrlicht)
    unsigned int pointerspacing_ = 100;  // Spacing between two visualizations (POV-Ray & Irrlicht)
    
    // Checkpointer usage
    bool useCheckpointer_ = true;  // Switches the checkpointer output of the simulation on and off
    
    // Checkpoint path
    boost::filesystem::path checkpoint_path_ = "checkpoints/";
};

} // namespace pe

extern "C" void step_simulation_();
void stepSimulation();
void setupParticleBench(MPI_Comm ex0);
void setupFSIBench(MPI_Comm ex0);
void setup2x2x2(MPI_Comm ex0);
void setupCyl(MPI_Comm ex0);
void setupBench(MPI_Comm ex0);
void setupKroupa(MPI_Comm ex0);
void setupCreep(MPI_Comm ex0);
void setupArchimedes(MPI_Comm ex0);
void setupDraftKissTumbBench(MPI_Comm ex0);
void setupGeneralInit(MPI_Comm ex0);
void setupSpan(MPI_Comm ex0);
#endif
