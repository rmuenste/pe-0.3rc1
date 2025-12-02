//=================================================================================================
/*!
 *  \file src/config/SimulationConfig.cpp
 *  \brief Implementation of the SimulationConfig class
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


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/config/SimulationConfig.h>
#include <fstream>
#include <stdexcept>
#if HAVE_JSON
#include <nlohmann/json.hpp>
#endif


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default constructor for SimulationConfig.
 *
 * Initializes all simulation parameters with default values suitable for typical
 * rigid body simulations. These defaults can be overridden using the setter methods.
 */
SimulationConfig::SimulationConfig()
    : timesteps_(15000)
    , stepsize_(0.001)
    , substeps_(1)
    , processesX_(3)
    , processesY_(3)
    , processesZ_(3)
    , cfdRank_(1)
    , seed_(12345)
    , verbose_(false)
    , vtk_(true)
    , visspacing_(50)
    , pointerspacing_(100)
    , useCheckpointer_(true)
    , checkpoint_path_("checkpoints/")
    , volumeFraction_(0.3)
    , benchRadius_(0.0015)
    , resume_(false)
    , packingMethod_(PackingMethod::Grid)
    , xyzFilePath_("")
    , particleDensity_(1.0)
    , fluidViscosity_(1.0)
    , fluidDensity_(1.0)
    , gravity_(0.0, 0.0, -9.81)
    , lubricationHysteresisDelta_(1e-3)
    , contactHysteresisDelta_(1e-9)
    , alphaImpulseCap_(1.0)
    , minEpsLub_(1e-8)
{
}
//=================================================================================================


//=================================================================================================
//
//  SINGLETON ACCESSOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Singleton accessor for SimulationConfig.
 *
 * \return Reference to the singleton SimulationConfig instance.
 *
 * This function implements the singleton pattern to ensure only one instance of
 * SimulationConfig exists throughout the application lifetime.
 */
SimulationConfig& SimulationConfig::getInstance() {
    static SimulationConfig instance;
    return instance;
}
//=================================================================================================




//=================================================================================================
//
//  CONFIGURATION LOADING
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Loads simulation configuration from a JSON file.
 *
 * \param fileName Path to the JSON configuration file.
 * \return void
 *
 * This function reads a JSON file and populates the SimulationConfig singleton with the
 * values found in the file. Only the parameters present in the JSON file are updated;
 * parameters not in the file retain their default values.
 *
 * \note Requires HAVE_JSON to be enabled during compilation. If JSON support is not available,
 *       this function does nothing (silent no-op).
 *
 * \exception std::runtime_error If the file cannot be opened.
 * \exception std::invalid_argument If the packingMethod_ value is invalid.
 */
void SimulationConfig::loadFromFile(const std::string &fileName) {

#if HAVE_JSON
    // Open the configuration file
    std::ifstream file(fileName);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open config file: " + fileName);
    }

    // Parse the JSON file using nlohmann::json
    nlohmann::json j;
    file >> j;

    // Retrieve the singleton instance of SimulationConfig
    SimulationConfig &config = getInstance();

    // Set time parameters
    if (j.contains("timesteps_"))
        config.setTimesteps(j["timesteps_"].get<size_t>());
    if (j.contains("stepsize_"))
        config.setStepsize(j["stepsize_"].get<real>());
    if (j.contains("substeps_"))
        config.setSubsteps(j["substeps_"].get<int>());

    // Set process parameters
    if (j.contains("processesX_"))
        config.setProcessesX(j["processesX_"].get<int>());
    if (j.contains("processesY_"))
        config.setProcessesY(j["processesY_"].get<int>());
    if (j.contains("processesZ_"))
        config.setProcessesZ(j["processesZ_"].get<int>());
    if (j.contains("cfdRank_"))
        config.setCfdRank(j["cfdRank_"].get<int>());

    // Set random number generator parameter
    if (j.contains("seed_"))
        config.setSeed(j["seed_"].get<size_t>());

    // Set verbose mode
    if (j.contains("verbose_"))
        config.setVerbose(j["verbose_"].get<bool>());

    // Set VTK visualization flag
    if (j.contains("vtk_"))
        config.setVtk(j["vtk_"].get<bool>());

    // Set visualization parameters
    if (j.contains("visspacing_"))
        config.setVisspacing(j["visspacing_"].get<unsigned int>());

    // Set the number of steps between checkpoints
    if (j.contains("pointerspacing_"))
        config.setPointerspacing(j["pointerspacing_"].get<unsigned int>());

    // Set checkpointer usage
    if (j.contains("useCheckpointer_"))
        config.setUseCheckpointer(j["useCheckpointer_"].get<bool>());

    // Set resume
    if (j.contains("resume_"))
        config.setResume(j["resume_"].get<bool>());

    // Set the checkpoint path (assuming the JSON key is a string)
    if (j.contains("checkpoint_path_"))
        config.setCheckpointPath(boost::filesystem::path(j["checkpoint_path_"].get<std::string>()));

    // Set simulation parameters
    if (j.contains("volumeFraction_"))
        config.setVolumeFraction(j["volumeFraction_"].get<real>());

    if (j.contains("benchRadius_"))
        config.setBenchRadius(j["benchRadius_"].get<real>());

    if (j.contains("packingMethod_"))
        config.setPackingMethod(parsePackingMethod(j["packingMethod_"].get<std::string>()));

    if (j.contains("xyzFilePath_"))
        config.setXyzFilePath(boost::filesystem::path(j["xyzFilePath_"].get<std::string>()));

    // Set particle density
    if (j.contains("particleDensity_"))
        config.setParticleDensity(j["particleDensity_"].get<real>());

    // Set fluid viscosity
    if (j.contains("fluidViscosity_"))
        config.setFluidViscosity(j["fluidViscosity_"].get<real>());

    // Set fluid density
    if (j.contains("fluidDensity_"))
        config.setFluidDensity(j["fluidDensity_"].get<real>());

    // Set lubrication hysteresis blend half-width
    if (j.contains("lubricationHysteresisDelta_"))
        config.setLubricationHysteresisDelta(j["lubricationHysteresisDelta_"].get<real>());

    // Set contact hysteresis blend half-width
    if (j.contains("contactHysteresisDelta_"))
        config.setContactHysteresisDelta(j["contactHysteresisDelta_"].get<real>());

    // Set lubrication impulse cap factor
    if (j.contains("alphaImpulseCap_"))
        config.setAlphaImpulseCap(j["alphaImpulseCap_"].get<real>());

    // Set lubrication gap regularization
    if (j.contains("minEpsLub_"))
        config.setMinEpsLub(j["minEpsLub_"].get<real>());

    // Set gravity vector
    if (j.contains("gravity_")) {
        if (j["gravity_"].is_array() && j["gravity_"].size() == 3) {
            Vec3 gravity(j["gravity_"][0].get<real>(),
                        j["gravity_"][1].get<real>(),
                        j["gravity_"][2].get<real>());
            config.setGravity(gravity);
        }
    }

#endif
}
//=================================================================================================

} // namespace pe
