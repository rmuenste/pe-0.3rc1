add_executable(mpidomain mpidomain.cpp)
target_include_directories(mpidomain PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(mpidomain)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( mpidomain      pe_static )
ELSE()
   ADD_DEPENDENCIES( mpidomain      pe_shared )
ENDIF()
