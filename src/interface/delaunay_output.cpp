#ifdef HAVE_CGAL
//=================================================================================================
//
//    CGAL DELAUNAY
//
//=================================================================================================
//=================================================================================================
// This functions computes the delaunay triangulation from the
// current particle positions and outputs it to a file
//=================================================================================================
void outputDelaunay(int timeStep) {

  namespace fs = boost::filesystem;

  std::vector<Vec3> localPoints;
  std::vector<pe::id_t> systemIDs;
  std::vector<int> localWallContacts;
  std::vector<real> localWallDistances;

  MPI_Comm cartcomm = theMPISystem()->getComm();
  real epsilon              = 2e-4;
  real radius2              = 0.01 - epsilon;

  for (unsigned int i(0); i < theCollisionSystem()->getBodyStorage().size(); i++) {
    World::SizeType widx = static_cast<World::SizeType>(i);
    BodyID body = world->getBody(static_cast<unsigned int>(widx));
    if(body->getType() == sphereType) { //} || body->getType() == capsuleType) {
      SphereID s = static_body_cast<Sphere>(body);
      radius2 = s->getRadius();
      Vec3 pos = s->getPosition();
      localPoints.push_back(pos);
      systemIDs.push_back(body->getSystemID());
      localWallContacts.push_back(body->wallContact_);
      localWallDistances.push_back(body->contactDistance_);
    }
  }

  int totalPoints(0);
  int maxPoints(0);
  int localSize = localPoints.size();
  MPI_Reduce(&localSize, &totalPoints, 1, MPI_INT, MPI_SUM, 0, mpisystem->getComm());

  //=================================================================
  // Simulation parameters
  //=================================================================
  real h = 0.001;
  real L = 0.1;
  real domainVol = L * L * L;
  real partVol = 4./3. * M_PI * std::pow(radius2, 3);
  real phi = (totalPoints * partVol)/domainVol * 100.0;

  //================================================================================================
  // We perform MPI communication of the particle positions here
  // Additionally we also want to communicate the particle systemIDs
  // and information about boundary contacts
  //
  // Since the number of particles can be different on each process we use the MPI_Gatherv function
  //================================================================================================
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  for(auto point : localPoints) {
     x.push_back(point[0]);
     y.push_back(point[1]);
     z.push_back(point[2]);
  }

  int local_size = x.size();

  // Get the size of the mpi world
  int size = mpisystem->getSize();
  std::vector<int> recvcounts(size);

  // Get the id of the current process
  int myrank = mpisystem->getRank(); 

  // Gather the size of data from each process on the root
  MPI_Gather(&local_size, 1, MPI_INT, recvcounts.data(), 1, MPI_INT, 0, cartcomm); 

  std::vector<int> displs(size, 0); // Displacement array

  if (myrank == 0) {
       // Calculate the displacement array for the gathered data
       for (int i = 1; i < size; ++i) {
           displs[i] = displs[i - 1] + recvcounts[i - 1];
           //std::cout << "Displacement " << i << " " << displs[i] << std::endl;
       }
  }   

  std::vector<double> all_points_x;
  std::vector<double> all_points_y;
  std::vector<double> all_points_z;
  std::vector<pe::id_t> allSystemIDs;
  std::vector<int> globalWallContacts;
  std::vector<real> globalWallDistances;
  int total_size;
  if (myrank == 0) {
     // Calculate the total size of the gathered data
     total_size = 0;
     for (int i = 0; i < recvcounts.size(); ++i) {
         total_size += recvcounts[i];
     }

     // Allocate space for the gathered data
     all_points_x.resize(total_size);      
     all_points_y.resize(total_size);      
     all_points_z.resize(total_size);      
     allSystemIDs.resize(total_size);      
     globalWallContacts.resize(total_size);      
     globalWallDistances.resize(total_size);      
  }

  //===============================================================================
  // Here we gather the x-y-z data on the root process
  //===============================================================================
  // Gather the data from each process into the all_data array
  MPI_Gatherv(x.data(), x.size(), MPI_DOUBLE,
              all_points_x.data(), recvcounts.data(), displs.data(), MPI_DOUBLE,
              0, cartcomm);              
  // Gather the data from each process into the all_data array
  MPI_Gatherv(y.data(), y.size(), MPI_DOUBLE,
              all_points_y.data(), recvcounts.data(), displs.data(), MPI_DOUBLE,
              0, cartcomm);              
  // Gather the data from each process into the all_data array
  MPI_Gatherv(z.data(), z.size(), MPI_DOUBLE,
              all_points_z.data(), recvcounts.data(), displs.data(), MPI_DOUBLE,
              0, cartcomm);              
  //===============================================================================
  // Here we gather the system ids on the root process
  //===============================================================================
  MPI_Gatherv(systemIDs.data(), systemIDs.size(), MPI_UNSIGNED_LONG_LONG,
              allSystemIDs.data(), recvcounts.data(), displs.data(), MPI_UNSIGNED_LONG_LONG,
              0, cartcomm);              
  //===============================================================================
  // Here we gather the wall contacts on the root process
  //===============================================================================
  MPI_Gatherv(localWallDistances.data(), localWallDistances.size(), MPI_DOUBLE,
              globalWallDistances.data(), recvcounts.data(), displs.data(), MPI_DOUBLE,
              0, cartcomm);              
  MPI_Gatherv(localWallContacts.data(), localWallContacts.size(), MPI_INT,
              globalWallContacts.data(), recvcounts.data(), displs.data(), MPI_INT,
              0, cartcomm);              

  pe_EXCLUSIVE_SECTION(0) {

  std::vector<std::vector<double>> cgalPoints;
  for (size_t idx(0); idx < all_points_x.size(); ++idx) {
    cgalPoints.push_back({all_points_x[idx], all_points_y[idx], all_points_z[idx]});
  }

  std::string directoryPath = "./delaunay";

  try {

    if(!fs::exists(directoryPath)) {
      // Create the directory
      fs::create_directory(directoryPath);
    }

  } catch (const fs::filesystem_error &ex) {
    std::cerr << "Error creating directory: " << ex.what() << std::endl;
  }

  std::stringstream ss;
  std::stringstream ss2;

  ss  << directoryPath << "/exchange_file_" << timeStep << ".dat";
  ss2 << directoryPath << "/delaunay_viz_"  << timeStep << ".vtk";

  cgal3d::outputDataToFile(all_points_x,
                    all_points_y,
                    all_points_z,
                    allSystemIDs,
                    globalWallContacts,
                    globalWallDistances,
                    cgalPoints,
                    L,
                    phi,
                    radius2,
                    ss.str(),
                    ss2.str() 
                    );

  }    

}
//=================================================================================================
#endif