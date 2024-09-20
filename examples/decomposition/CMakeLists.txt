add_executable(decomposition decomposition.cpp)
target_include_directories(decomposition PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(decomposition)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( decomposition      pe_static )
ELSE()
   ADD_DEPENDENCIES( decomposition      pe_shared )
ENDIF()