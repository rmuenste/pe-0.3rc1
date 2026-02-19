# Serial Examples
add_subdirectory(body_removal)  # Example showing body removal during simulation
add_subdirectory(box_domain)
add_subdirectory(cylinder)
add_subdirectory(decomposition)
add_subdirectory(detection)
add_subdirectory(kdop_example)  # Example showing k-DOP extraction from TriMeshDopBoundary
add_subdirectory(math_tests)
add_subdirectory(rotation_collision)  # Example showing translation-fixed vs fully-fixed rigid bodies
add_subdirectory(simple_granular)
add_subdirectory(triangle_mesh)
add_subdirectory(usage_demo)
add_subdirectory(simple_demo)
add_subdirectory(basic_lubrication)
add_subdirectory(lubrication_demo)
add_subdirectory(capsule_spin)
add_subdirectory(vtk_shapes_ascii)
add_subdirectory(vtk_writer_test)  # VTK Writer verification test
add_subdirectory(sphere_column_serial)  # Serial sphere column with ShortRangeRepulsion solver

# CGAL Examples
if( CGAL )
    add_subdirectory(cgal_examples)  # Example showing CGAL oriented bounding box functionality
    add_subdirectory(convex_hull)  # Example showing CGAL convex hull computation for triangle meshes
    add_subdirectory(mesh_spin)  # Example showing mesh rotation with DistanceMap collision detection
endif( CGAL )

# MPI Examples
if( MPI )
    add_subdirectory(decomposition_geometry)
    add_subdirectory(mpi3periodic)
    add_subdirectory(mpibench)
    add_subdirectory(mpiboxes)
    add_subdirectory(mpicapsules)
    add_subdirectory(mpidem)
    add_subdirectory(mpidomain)
    add_subdirectory(mpiperiodic)
    add_subdirectory(mpiperiodic_dev)
    add_subdirectory(mpiplanedecomposition)
    add_subdirectory(mpispheres)
    add_subdirectory(mpiunion)
    add_subdirectory(rectigrid_demo)
    add_subdirectory(trimeshdop_demo) # TriMeshDopBoundary example using OBJ files for domain decomposition
endif( MPI )
