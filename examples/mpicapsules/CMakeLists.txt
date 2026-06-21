add_executable(mpicapsules mpicapsules.cpp)
target_include_directories(mpicapsules PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(mpicapsules PRIVATE 
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
)