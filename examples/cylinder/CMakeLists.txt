add_executable(cylinder cylinder_test.cpp)
target_include_directories(cylinder PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(cylinder)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( cylinder      pe_static )
ELSE()
   ADD_DEPENDENCIES( cylinder      pe_shared )
ENDIF()