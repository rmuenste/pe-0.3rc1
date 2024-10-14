add_executable(mpiplanedecomposition mpiplanedecomposition.cpp)
target_include_directories(mpiplanedecomposition PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(mpiplanedecomposition PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)