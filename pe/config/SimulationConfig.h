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
#include <string>
#include <vector>


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
        Grid,             //!< Regular grid packing (spheres only)
        MixedGrid,        //!< Mixed shape grid packing (spheres, boxes, capsules)
        TriangleMeshGrid, //!< Triangle mesh grid packing (cones from cone.obj)
        External,         //!< Load from external file
        None              //!< No initial packing
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
        if (method == "mixed_grid") return PackingMethod::MixedGrid;
        if (method == "tm_grid") return PackingMethod::TriangleMeshGrid;
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
    int getSubsteps() const { return substeps_; }
    void setTimesteps(size_t value) { timesteps_ = value; }
    void setStepsize(real value) { stepsize_ = value; }
    void setSubsteps(int value) { substeps_ = value; }
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
    int getCfdRank() const { return cfdRank_; }
    void setCfdRank(int value) { cfdRank_ = value; }
    const Vec3& getCfdDomainMin() const { return cfdDomainMin_; }
    const Vec3& getCfdDomainMax() const { return cfdDomainMax_; }
    void setCfdDomainMin(const Vec3& value) { cfdDomainMin_ = value; }
    void setCfdDomainMax(const Vec3& value) { cfdDomainMax_ = value; }
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
    const std::string& getResumeCheckpointFile() const { return resumeCheckpointFile_; }
    void setResumeCheckpointFile(const std::string& file) { resumeCheckpointFile_ = file; }

    // Resume expectations. All opt-in: an unset expectation is not checked, a set one that is
    // violated aborts the run. Sentinels mean "unset" -- a negative step index and an empty tag
    // are not otherwise meaningful, and a driver that wants t = 0 checked must say so with a
    // tolerance rather than relying on a default.
    bool getResumeExpectedTimeSet() const { return resumeExpectedTimeSet_; }
    real getResumeExpectedTime() const { return resumeExpectedTime_; }
    void setResumeExpectedTime(real value) { resumeExpectedTime_ = value; resumeExpectedTimeSet_ = true; }
    real getResumeTimeToleranceSteps() const { return resumeTimeToleranceSteps_; }
    void setResumeTimeToleranceSteps(real value) { resumeTimeToleranceSteps_ = value; }
    long long getResumeExpectedStep() const { return resumeExpectedStep_; }
    void setResumeExpectedStep(long long value) { resumeExpectedStep_ = value; }
    const std::string& getResumeExpectedTag() const { return resumeExpectedTag_; }
    void setResumeExpectedTag(const std::string& tag) { resumeExpectedTag_ = tag; }
    //@}
    //**************************************************************************************

    //**Simulation parameters***************************************************************
    /*!\name Simulation parameters */
    //@{
    real getVolumeFraction() const { return volumeFraction_; }
    void setVolumeFraction(real value) { volumeFraction_ = value; }
    real getBenchRadius() const { return benchRadius_; }
    void setBenchRadius(real value) { benchRadius_ = value; }
    const std::string& getSeedMode() const { return seedMode_; }
    void setSeedMode(const std::string& value) { seedMode_ = value; }
    real getSeedMinGap() const { return seedMinGap_; }
    void setSeedMinGap(real value) { seedMinGap_ = value; }
    bool getSeedAllowContact() const { return seedAllowContact_; }
    void setSeedAllowContact(bool value) { seedAllowContact_ = value; }
    const std::string& getSeedDomain() const { return seedDomain_; }
    void setSeedDomain(const std::string& value) { seedDomain_ = value; }
    const Vec3& getSeedCylinderCenter() const { return seedCylinderCenter_; }
    void setSeedCylinderCenter(const Vec3& value) { seedCylinderCenter_ = value; }
    real getSeedCylinderRadius() const { return seedCylinderRadius_; }
    void setSeedCylinderRadius(real value) { seedCylinderRadius_ = value; }
    const std::string& getSeedCylinderAxis() const { return seedCylinderAxis_; }
    void setSeedCylinderAxis(const std::string& value) { seedCylinderAxis_ = value; }
    bool getPeriodicX() const { return periodicX_; }
    void setPeriodicX(bool value) { periodicX_ = value; }
    bool getPeriodicY() const { return periodicY_; }
    void setPeriodicY(bool value) { periodicY_ = value; }
    bool getPeriodicZ() const { return periodicZ_; }
    void setPeriodicZ(bool value) { periodicZ_ = value; }
    real getFluidizationSpacingFactor() const { return fluidizationSpacingFactor_; }
    void setFluidizationSpacingFactor(real value) { fluidizationSpacingFactor_ = value; }
    const Vec3& getBenchStartPosition() const { return benchStartPosition_; }
    void setBenchStartPosition(const Vec3& value) { benchStartPosition_ = value; }
    bool getBenchUseConfigStart() const { return benchUseConfigStart_; }
    void setBenchUseConfigStart(bool value) { benchUseConfigStart_ = value; }
    const Vec3& getInitialParticleVelocity() const { return initialParticleVelocity_; }
    void setInitialParticleVelocity(const Vec3& value) { initialParticleVelocity_ = value; }
    //@}
    //**************************************************************************************

    //**Domain boundary parameters*********************************************************
    /*!\name Domain boundary parameters */
    //@{
    bool getDomainBoundaryEnabled() const { return domainBoundaryEnabled_; }
    void setDomainBoundaryEnabled(bool value) { domainBoundaryEnabled_ = value; }
    const boost::filesystem::path& getDomainBoundaryFilePath() const { return domainBoundaryFilePath_; }
    void setDomainBoundaryFilePath(const boost::filesystem::path& path) { domainBoundaryFilePath_ = path; }
    const Vec3& getDomainBoundaryPosition() const { return domainBoundaryPosition_; }
    void setDomainBoundaryPosition(const Vec3& value) { domainBoundaryPosition_ = value; }
    bool getDomainBoundaryFixed() const { return domainBoundaryFixed_; }
    void setDomainBoundaryFixed(bool value) { domainBoundaryFixed_ = value; }
    bool getDomainBoundaryVisible() const { return domainBoundaryVisible_; }
    void setDomainBoundaryVisible(bool value) { domainBoundaryVisible_ = value; }
    bool getDomainBoundaryDistanceMapEnabled() const { return domainBoundaryDistanceMapEnabled_; }
    void setDomainBoundaryDistanceMapEnabled(bool value) { domainBoundaryDistanceMapEnabled_ = value; }
    int getDomainBoundaryDistanceMapResolution() const { return domainBoundaryDistanceMapResolution_; }
    void setDomainBoundaryDistanceMapResolution(int value) { domainBoundaryDistanceMapResolution_ = value; }
    int getDomainBoundaryDistanceMapTolerance() const { return domainBoundaryDistanceMapTolerance_; }
    void setDomainBoundaryDistanceMapTolerance(int value) { domainBoundaryDistanceMapTolerance_ = value; }
    bool getDomainBoundaryInvertDistanceMap() const { return domainBoundaryInvertDistanceMap_; }
    void setDomainBoundaryInvertDistanceMap(bool value) { domainBoundaryInvertDistanceMap_ = value; }
    bool getDomainBoundaryWriteDistanceMapVti() const { return domainBoundaryWriteDistanceMapVti_; }
    void setDomainBoundaryWriteDistanceMapVti(bool value) { domainBoundaryWriteDistanceMapVti_ = value; }
    const boost::filesystem::path& getDomainBoundaryDistanceMapVtiFile() const { return domainBoundaryDistanceMapVtiFile_; }
    void setDomainBoundaryDistanceMapVtiFile(const boost::filesystem::path& path) { domainBoundaryDistanceMapVtiFile_ = path; }
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
    real getLubricationHysteresisDelta() const { return lubricationHysteresisDelta_; }
    void setLubricationHysteresisDelta(real value) { lubricationHysteresisDelta_ = value; }
    real getContactHysteresisDelta() const { return contactHysteresisDelta_; }
    void setContactHysteresisDelta(real value) { contactHysteresisDelta_ = value; }
    real getAlphaImpulseCap() const { return alphaImpulseCap_; }
    void setAlphaImpulseCap(real value) { alphaImpulseCap_ = value; }
    real getMinEpsLub() const { return minEpsLub_; }
    void setMinEpsLub(real value) { minEpsLub_ = value; }
    bool getLubricationEnabled() const { return lubricationEnabled_; }
    void setLubricationEnabled(bool value) { lubricationEnabled_ = value; }
    const std::string& getLubricationModel() const { return lubricationModel_; }
    void setLubricationModel(const std::string& value) {
        if (value != "kroupa2016" && value != "legacy")
            throw std::invalid_argument("Unknown lubricationModel_: " + value +
                                        " (expected \"kroupa2016\" or \"legacy\")");
        lubricationModel_ = value;
    }
    const std::string& getLubricationIntegration() const { return lubricationIntegration_; }
    void setLubricationIntegration(const std::string& value) {
        if (value != "semi-implicit" && value != "explicit-capped")
            throw std::invalid_argument("Unknown lubricationIntegration_: " + value +
                                        " (expected \"semi-implicit\" or \"explicit-capped\")");
        lubricationIntegration_ = value;
    }
    bool getLubricationTangential() const { return lubricationTangential_; }
    void setLubricationTangential(bool value) { lubricationTangential_ = value; }
    bool getLubricationTwisting() const { return lubricationTwisting_; }
    void setLubricationTwisting(bool value) { lubricationTwisting_ = value; }
    bool getLubricationSlipCorrection() const { return lubricationSlipCorrection_; }
    void setLubricationSlipCorrection(bool value) { lubricationSlipCorrection_ = value; }
    bool getLubricationOnSeparation() const { return lubricationOnSeparation_; }
    void setLubricationOnSeparation(bool value) { lubricationOnSeparation_ = value; }
    bool getLubricationWallTerms() const { return lubricationWallTerms_; }
    void setLubricationWallTerms(bool value) { lubricationWallTerms_ = value; }
    real getLubricationEpsCritical() const { return lubricationEpsCritical_; }
    void setLubricationEpsCritical(real value) { lubricationEpsCritical_ = value; }
    real getLubricationCutoffFactor() const { return lubricationCutoffFactor_; }
    void setLubricationCutoffFactor(real value) { lubricationCutoffFactor_ = value; }
    real getLubricationMeshClampFactor() const { return lubricationMeshClampFactor_; }
    void setLubricationMeshClampFactor(real value) { lubricationMeshClampFactor_ = value; }
    bool getLubricationAabbInflation() const { return lubricationAabbInflation_; }
    void setLubricationAabbInflation(bool value) { lubricationAabbInflation_ = value; }
    real getLubricationCutoff() const { return lubricationCutoff_; }
    void setLubricationCutoff(real value) { lubricationCutoff_ = value; }
    real getLubricationSlipLength() const { return lubricationSlipLength_; }
    void setLubricationSlipLength(real value) { lubricationSlipLength_ = value; }
    bool getZWallsEnabled() const { return zWallsEnabled_; }
    void setZWallsEnabled(bool value) { zWallsEnabled_ = value; }
    real getZWallVelocityTop() const { return zWallVelocityTop_; }
    void setZWallVelocityTop(real value) { zWallVelocityTop_ = value; }
    real getZWallVelocityBottom() const { return zWallVelocityBottom_; }
    void setZWallVelocityBottom(real value) { zWallVelocityBottom_ = value; }
    real getRestitution() const { return restitution_; }
    void setRestitution(real value) { restitution_ = value; }
    real getStaticFriction() const { return staticFriction_; }
    void setStaticFriction(real value) { staticFriction_ = value; }
    real getDynamicFriction() const { return dynamicFriction_; }
    void setDynamicFriction(real value) { dynamicFriction_ = value; }
    //@}
    //**************************************************************************************

    //**Serial post-step feature parameters************************************************
    /*!\name Serial post-step feature switches and parameters */
    //@{
    bool getSerialEnableEscapeReinsertion() const { return serialEnableEscapeReinsertion_; }
    void setSerialEnableEscapeReinsertion(bool value) { serialEnableEscapeReinsertion_ = value; }
    bool getSerialEnableStuckDiagnostics() const { return serialEnableStuckDiagnostics_; }
    void setSerialEnableStuckDiagnostics(bool value) { serialEnableStuckDiagnostics_ = value; }
    real getSerialReinsertionSafetyMargin() const { return serialReinsertionSafetyMargin_; }
    void setSerialReinsertionSafetyMargin(real value) { serialReinsertionSafetyMargin_ = value; }
    const std::vector<Vec3>& getSerialReinsertionSafePositions() const { return serialReinsertionSafePositions_; }
    void setSerialReinsertionSafePositions(const std::vector<Vec3>& positions) { serialReinsertionSafePositions_ = positions; }
    int getSerialStuckDetectionWindow() const { return serialStuckDetectionWindow_; }
    void setSerialStuckDetectionWindow(int value) { serialStuckDetectionWindow_ = value; }
    real getSerialStuckDisplacementThreshold() const { return serialStuckDisplacementThreshold_; }
    void setSerialStuckDisplacementThreshold(real value) { serialStuckDisplacementThreshold_ = value; }
    real getSerialStuckWallDistanceThreshold() const { return serialStuckWallDistanceThreshold_; }
    void setSerialStuckWallDistanceThreshold(real value) { serialStuckWallDistanceThreshold_ = value; }
    bool getSerialEnableSpherePositionLog() const { return serialEnableSpherePositionLog_; }
    void setSerialEnableSpherePositionLog(bool value) { serialEnableSpherePositionLog_ = value; }
    unsigned int getSerialSpherePositionLogSpacing() const { return serialSpherePositionLogSpacing_; }
    void setSerialSpherePositionLogSpacing(unsigned int value) { serialSpherePositionLogSpacing_ = value; }
    //@}
    //**************************************************************************************

    //**Centerline parameters***************************************************************
    /*!\name Centerline parameters for stuck particle diagnostics */
    //@{
    const std::vector<Vec3>& getCenterlineVertices() const { return centerlineVertices_; }
    void setCenterlineVertices(const std::vector<Vec3>& vertices) { centerlineVertices_ = vertices; }
    real getTotalCenterlineLength() const { return totalCenterlineLength_; }
    void setTotalCenterlineLength(real length) { totalCenterlineLength_ = length; }
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
    int substeps_;               //!< Number of substeps per main timestep (for substepping integration)

    // Process parameters (for MPI domain decomposition)
    int processesX_;             //!< Number of processes in x-direction
    int processesY_;             //!< Number of processes in y-direction
    int processesZ_;             //!< Number of processes in z-direction
    int cfdRank_;                //!< Designated CFD rank owning the authoritative particle state
    Vec3 cfdDomainMin_;          //!< Global minimum corner of the CFD mesh domain
    Vec3 cfdDomainMax_;          //!< Global maximum corner of the CFD mesh domain

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
    std::string resumeCheckpointFile_; //!< Checkpoint base name/path used for resume loading
    bool resumeExpectedTimeSet_;       //!< Whether resumeExpectedTime_ carries a driver expectation
    real resumeExpectedTime_;          //!< Simulation time the driver expects the checkpoint to hold
    real resumeTimeToleranceSteps_;    //!< Allowed time deviation, in units of the checkpoint stepSize
    long long resumeExpectedStep_;     //!< Step index the driver expects; negative means unset
    std::string resumeExpectedTag_;    //!< Pairing tag the driver expects; empty means unset

    // Simulation parameters
    real volumeFraction_;        //!< Volume fraction for particle packing
    real benchRadius_;           //!< Radius of the benchmark geometry
    std::string seedMode_;       //!< EL validation seeding mode: file or random
    real seedMinGap_;            //!< Minimum surface-to-surface seed gap; negative means setup default
    bool seedAllowContact_;      //!< Allow intentionally touching/overlapping seeds (skips the min-gap guard; test cases)
    bool periodicX_;             //!< Periodic wrap-around domain connectivity in x (MPI build only)
    bool periodicY_;             //!< Periodic wrap-around domain connectivity in y (MPI build only)
    bool periodicZ_;             //!< Periodic wrap-around domain connectivity in z (MPI build only)
    std::string seedDomain_;     //!< Random seed support domain: box or cylinder
    Vec3 seedCylinderCenter_;    //!< Center point for cylindrical random seed support
    real seedCylinderRadius_;    //!< Radius for cylindrical random seed support
    std::string seedCylinderAxis_; //!< Axis for cylindrical random seed support
    real fluidizationSpacingFactor_; //!< Gap factor relative to radius for fluidization grid packing
    Vec3 benchStartPosition_;    //!< Initial position of the benchmark geometry
    bool benchUseConfigStart_;   //!< Opt-in: bench setups take the start position from benchStartPosition_ instead of their legacy hard-coded value
    Vec3 initialParticleVelocity_; //!< Optional initial velocity assigned to created particles

    // Domain boundary parameters
    bool domainBoundaryEnabled_; //!< Enable ATC triangle mesh domain boundary setup
    boost::filesystem::path domainBoundaryFilePath_; //!< Triangle mesh file used for ATC domain boundary
    Vec3 domainBoundaryPosition_; //!< World-frame position applied to the ATC domain boundary
    bool domainBoundaryFixed_; //!< Whether the ATC domain boundary is fixed
    bool domainBoundaryVisible_; //!< Whether the ATC domain boundary is visible to PE writers
    bool domainBoundaryDistanceMapEnabled_; //!< Enable domain boundary distance map construction
    int domainBoundaryDistanceMapResolution_; //!< Distance map resolution parameter
    int domainBoundaryDistanceMapTolerance_; //!< Distance map tolerance parameter
    bool domainBoundaryInvertDistanceMap_; //!< Invert distance map signs/normals for domain boundary use
    bool domainBoundaryWriteDistanceMapVti_; //!< Write distance map VTI for inspection
    boost::filesystem::path domainBoundaryDistanceMapVtiFile_; //!< Output VTI filename

    // Packing parameters
    PackingMethod packingMethod_; //!< Particle packing method
    boost::filesystem::path xyzFilePath_; //!< Path to external particle position file

    // Physical parameters
    real particleDensity_;       //!< Particle material density
    real fluidViscosity_;        //!< Fluid dynamic viscosity
    real fluidDensity_;          //!< Fluid density
    Vec3 gravity_;               //!< Gravity vector
    real lubricationHysteresisDelta_; //!< Half-width of lubrication blend band
    real contactHysteresisDelta_; //!< Half-width of contact blend band
    real alphaImpulseCap_;       //!< Max impulse cap factor for lubrication (explicit-capped scheme)
    real minEpsLub_;             //!< Regularization epsilon for lubrication gap (numerical floor)
    bool lubricationEnabled_;    //!< Master switch for lubrication forces
    std::string lubricationModel_; //!< Force model: "kroupa2016" or "legacy"
    std::string lubricationIntegration_; //!< Scheme: "semi-implicit" or "explicit-capped"
    bool lubricationTangential_; //!< Sliding force + sliding torque terms
    bool lubricationTwisting_;   //!< Twisting torque term
    bool lubricationSlipCorrection_; //!< Vinogradova slip correction f*
    bool lubricationOnSeparation_; //!< Resist separating motion (suction) as well
    bool lubricationWallTerms_;  //!< Dedicated wall resqistance set for sphere-plane pairs
    real lubricationEpsCritical_; //!< eps_c = h_c/a_ref saturation/slip-length scale
    real lubricationCutoffFactor_; //!< eps_cut = h_cut/a_ref outer cutoff; 0 = legacy absolute
    real lubricationMeshClampFactor_; //!< Mesh clamp c: h_cut capped at c*dx_CFD; 0 = off
    bool lubricationAabbInflation_; //!< Grow AABBs by the lubrication cutoff
    real lubricationCutoff_;     //!< Surface-gap trigger distance for lubrication pairs (EL solver)
    real lubricationSlipLength_; //!< Slip length h_c of the Vinogradova f* correction
    bool zWallsEnabled_;         //!< Create global z-planes (Couette walls) in the EL setup
    real zWallVelocityTop_;      //!< x-velocity of the top wall (z = zmax)
    real zWallVelocityBottom_;   //!< x-velocity of the bottom wall (z = zmin)
    real restitution_;           //!< Contact material restitution coefficient
    real staticFriction_;        //!< Contact material static friction coefficient
    real dynamicFriction_;       //!< Contact material dynamic friction coefficient

    // Serial post-step features
    bool serialEnableEscapeReinsertion_; //!< Enable domain escape handling and reinsertion in serial stepping
    bool serialEnableStuckDiagnostics_; //!< Enable stuck particle diagnostics in serial stepping
    real serialReinsertionSafetyMargin_; //!< Additional spacing used when testing safe reinsertion positions
    std::vector<Vec3> serialReinsertionSafePositions_; //!< Candidate reinsertion positions for escaped particles
    int serialStuckDetectionWindow_; //!< Number of main timesteps considered for stuck diagnostics
    real serialStuckDisplacementThreshold_; //!< Max displacement across the detection window to classify a particle as stuck
    real serialStuckWallDistanceThreshold_; //!< Distance-to-wall threshold used for near-wall stuck diagnostics
    bool serialEnableSpherePositionLog_; //!< Enable writing per-timestep sphere position snapshots in serial stepping
    unsigned int serialSpherePositionLogSpacing_; //!< Main-timestep spacing for sphere position snapshot output

    // Centerline data for stuck particle diagnostics
    std::vector<Vec3> centerlineVertices_; //!< Centerline vertices for tube geometry
    real totalCenterlineLength_;           //!< Precomputed total length of centerline
    //@}
    //**************************************************************************************
};

} // namespace pe

#endif
