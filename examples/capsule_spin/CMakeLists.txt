add_executable(capsule_spin capsule_spin.cpp)
target_include_directories(capsule_spin PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(capsule_spin PRIVATE 
  $<$<STREQUAL:${PE_LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${PE_LIBRARY_TYPE},SHARED>:pe_shared>
)
