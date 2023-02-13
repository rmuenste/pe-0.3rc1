add_executable(mpicapsules mpicapsules.cpp)
target_include_directories(mpicapsules PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(mpicapsules)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( mpicapsules      pe_static )
ELSE()
   ADD_DEPENDENCIES( mpicapsules      pe_shared )
ENDIF()
