add_executable(math_tests math_tests.cpp)
target_include_directories(math_tests PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(math_tests)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( math_tests      pe_static )
ELSE()
   ADD_DEPENDENCIES( math_tests      pe_shared )
ENDIF()