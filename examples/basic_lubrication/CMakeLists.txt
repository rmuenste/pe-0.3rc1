add_executable(basic_lubrication basic_lubrication.cpp)
target_include_directories(basic_lubrication PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(basic_lubrication PRIVATE
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
)
