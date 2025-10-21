//=================================================================================================
/*!
 *  \file pe/config/SimulationConfig.h
 *  \brief Simulation configuration class for managing simulation parameters
 *
 *  Copyright (C) 2023 Raphael Muenster
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================

#ifndef _PE_CONFIG_SIMULATIONCONFIG_H_
#define _PE_CONFIG_SIMULATIONCONFIG_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/config/Precision.h>
#include <pe/math/Vector3.h>
#include <boost/filesystem.hpp>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Simulation configuration class to encapsulate all simulation parameters
 * \ingroup config
 *
 * The SimulationConfig class provides a singleton pattern to manage global simulation
 * parameters. This class is available in all build modes (MPI, serial PE, no-PE) and
 * provides centralized configuration management for:
 * - Time stepping parameters
 * - Domain decomposition settings (for MPI builds)
 * - Visualization and output settings
 * - Physical parameters (gravity, fluid properties, particle properties)
 * - Checkpoint and resume functionality
 *
 * Usage example:
 * \code
 * auto& config = SimulationConfig::getInstance();
 * config.setTimesteps(10000);
 * config.setGravity(Vec3(0.0, 0.0, -9.81));
 * real viscosity = config.getFluidViscosity();
 * \endcode
 */
class SimulationConfig {
public:

    //**Packing method enumeration**********************************************************
    /*!\brief Enumeration for particle packing methods
     *
     * Defines how particles are initially arranged in the simulation domain.
     */
    enum PackingMethod {
        Grid,      //!< Regular grid packing
        External,  //!< Load from external file
        None       //!< No initial packing
    };
    //**************************************************************************************

    //**Singleton accessor******************************************************************
    /*!\name Singleton accessor */
    //@{
    static SimulationConfig& getInstance();
    //@}
    //**************************************************************************************

    //**Configuration loading***************************************************************
    /*!\name Configuration loading */
    //@{
    static void loadFromFile(const std::string& fileName);
    //@}
    //**************************************************************************************

    //**Utility functions*******************************************************************
    /*!\name Utility functions */
    //@{
    static PackingMethod parsePackingMethod(const std::string& method) {
        if (method == "grid") return PackingMethod::Grid;
        if (method == "external") return PackingMethod::External;
        if (method == "none") return PackingMethod::None;
        throw std::invalid_argument("Unknown packing method: " + method);
    }
    //@}
    //**************************************************************************************

    //**Time parameters*********************************************************************
    /*!\name Time parameters */
    //@{
    size_t getTimesteps() const { return timesteps_; }
    real getStepsize() const { return stepsize_; }
    void setTimesteps(size_t value) { timesteps_ = value; }
    void setStepsize(real value) { stepsize_ = value; }
    //@}
    //**************************************************************************************

    //**Process parameters******************************************************************
    /*!\name Process parameters */
    //@{
    int getProcessesX() const { return processesX_; }
    int getProcessesY() const { return processesY_; }
    int getProcessesZ() const { return processesZ_; }
    void setProcessesX(int value) { processesX_ = value; }
    void setProcessesY(int value) { processesY_ = value; }
    void setProcessesZ(int value) { processesZ_ = value; }
    int getPx() const { return processesX_; }
    int getPy() const { return processesY_; }
    int getPz() const { return processesZ_; }
    //@}
    //**************************************************************************************

    //**Random number generator parameters**************************************************
    /*!\name Random number generator parameters */
    //@{
    size_t getSeed() const { return seed_; }
    void setSeed(size_t value) { seed_ = value; }
    //@}
    //**************************************************************************************

    //**Output and debugging parameters*****************************************************
    /*!\name Output and debugging parameters */
    //@{
    bool getVerbose() const { return verbose_; }
    void setVerbose(bool value) { verbose_ = value; }
    bool getVtk() const { return vtk_; }
    void setVtk(bool value) { vtk_ = value; }
    //@}
    //**************************************************************************************

    //**Visualization parameters************************************************************
    /*!\name Visualization parameters */
    //@{
    unsigned int getVisspacing() const { return visspacing_; }
    unsigned int getPointerspacing() const { return pointerspacing_; }
    void setVisspacing(unsigned int value) { visspacing_ = value; }
    void setPointerspacing(unsigned int value) { pointerspacing_ = value; }
    //@}
    //**************************************************************************************

    //**Checkpoint parameters***************************************************************
    /*!\name Checkpoint parameters */
    //@{
    bool getUseCheckpointer() const { return useCheckpointer_; }
    void setUseCheckpointer(bool value) { useCheckpointer_ = value; }
    const boost::filesystem::path& getCheckpointPath() const { return checkpoint_path_; }
    void setCheckpointPath(const boost::filesystem::path& path) { checkpoint_path_ = path; }
    bool getResume() const { return resume_; }
    void setResume(bool value) { resume_ = value; }
    //@}
    //**************************************************************************************

    //**Simulation parameters***************************************************************
    /*!\name Simulation parameters */
    //@{
    real getVolumeFraction() const { return volumeFraction_; }
    void setVolumeFraction(real value) { volumeFraction_ = value; }
    real getBenchRadius() const { return benchRadius_; }
    void setBenchRadius(real value) { benchRadius_ = value; }
    //@}
    //**************************************************************************************

    //**Packing parameters******************************************************************
    /*!\name Packing parameters */
    //@{
    void setPackingMethod(PackingMethod method) { packingMethod_ = method; }
    PackingMethod getPackingMethod() const { return packingMethod_; }
    void setXyzFilePath(const boost::filesystem::path& path) { xyzFilePath_ = path; }
    const boost::filesystem::path& getXyzFilePath() const { return xyzFilePath_; }
    //@}
    //**************************************************************************************

    //**Physical parameters*****************************************************************
    /*!\name Physical parameters */
    //@{
    real getParticleDensity() const { return particleDensity_; }
    void setParticleDensity(real value) { particleDensity_ = value; }
    real getFluidViscosity() const { return fluidViscosity_; }
    void setFluidViscosity(real value) { fluidViscosity_ = value; }
    real getFluidDensity() const { return fluidDensity_; }
    void setFluidDensity(real value) { fluidDensity_ = value; }
    const Vec3& getGravity() const { return gravity_; }
    void setGravity(const Vec3& value) { gravity_ = value; }
    //@}
    //**************************************************************************************

private:
    //**Constructor*************************************************************************
    /*!\name Constructor */
    //@{
    SimulationConfig();
    //@}
    //**************************************************************************************

    //**Member variables********************************************************************
    /*!\name Member variables */
    //@{
    // Time parameters
    size_t timesteps_;           //!< Number of time steps for the simulation
    real stepsize_;              //!< Size of a single time step

    // Process parameters (for MPI domain decomposition)
    int processesX_;             //!< Number of processes in x-direction
    int processesY_;             //!< Number of processes in y-direction
    int processesZ_;             //!< Number of processes in z-direction

    // Random number generator parameters
    size_t seed_;                //!< Seed for random number generator

    // Output and debugging
    bool verbose_;               //!< Enable verbose output
    bool vtk_;                   //!< Enable VTK visualization output

    // Visualization parameters
    unsigned int visspacing_;    //!< Spacing between two visualizations (POV-Ray & Irrlicht)
    unsigned int pointerspacing_;//!< Spacing between pointer outputs

    // Checkpoint parameters
    bool useCheckpointer_;       //!< Enable checkpointer output
    boost::filesystem::path checkpoint_path_; //!< Path for checkpoint files
    bool resume_;                //!< Resume from checkpoint

    // Simulation parameters
    real volumeFraction_;        //!< Volume fraction for particle packing
    real benchRadius_;           //!< Radius of the benchmark geometry

    // Packing parameters
    PackingMethod packingMethod_; //!< Particle packing method
    boost::filesystem::path xyzFilePath_; //!< Path to external particle position file

    // Physical parameters
    real particleDensity_;       //!< Particle material density
    real fluidViscosity_;        //!< Fluid dynamic viscosity
    real fluidDensity_;          //!< Fluid density
    Vec3 gravity_;               //!< Gravity vector
    //@}
    //**************************************************************************************
};

} // namespace pe

#endif
