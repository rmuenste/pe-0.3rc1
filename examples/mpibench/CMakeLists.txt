add_executable(mpibench mpibench.cpp)
target_include_directories(mpibench PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(mpibench)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( mpibench      pe_static )
ELSE()
   ADD_DEPENDENCIES( mpibench      pe_shared )
ENDIF()
