add_executable(mpiplanedecomposition mpiplanedecomposition.cpp)
target_include_directories(mpiplanedecomposition PUBLIC ${FC_CORE_INCLUDE_DIRS})
target_link_libraries(mpiplanedecomposition)

# Dependencies
IF( LIBRARY_TYPE STREQUAL STATIC )
   ADD_DEPENDENCIES( mpiplanedecomposition      pe_static )
ELSE()
   ADD_DEPENDENCIES( mpiplanedecomposition      pe_shared )
ENDIF()
