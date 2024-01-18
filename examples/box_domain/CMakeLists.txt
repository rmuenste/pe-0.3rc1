add_executable(box_domain box_domain.cpp)
target_include_directories(box_domain PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(box_domain)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( box_domain      pe_static )
ELSE()
   ADD_DEPENDENCIES( box_domain      pe_shared )
ENDIF()