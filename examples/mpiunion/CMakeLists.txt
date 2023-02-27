add_executable(mpiunion mpiunion.cpp)
target_include_directories(mpiunion PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(mpiunion)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( mpiunion      pe_static )
ELSE()
   ADD_DEPENDENCIES( mpiunion      pe_shared )
ENDIF()
