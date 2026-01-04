add_executable(capsule_spin capsule_spin.cpp)
target_include_directories(capsule_spin PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(capsule_spin PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)
