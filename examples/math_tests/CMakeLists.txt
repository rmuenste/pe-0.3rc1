add_executable(math_tests math_tests.cpp)
target_include_directories(math_tests PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(math_tests PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)