add_executable(simple_granular simple_granular.cpp)
target_include_directories(simple_granular PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(simple_granular)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( simple_granular      pe_static )
ELSE()
   ADD_DEPENDENCIES( simple_granular      pe_shared )
ENDIF()