#include <pe/core/MPI.h>
extern "C" void step_simulation_();
void stepSimulation();


void setupParticleBench(MPI_Comm ex0);
void setup2x2x2(MPI_Comm ex0);
void setupCyl(MPI_Comm ex0);
void setupBench(MPI_Comm ex0);
void setupDraftKissTumbBench(MPI_Comm ex0);