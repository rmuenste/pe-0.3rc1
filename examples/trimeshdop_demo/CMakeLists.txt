add_executable(trimeshdop_demo trimeshdop_demo.cpp)
target_include_directories(trimeshdop_demo PRIVATE ${PE_EXAMPLES_INCLUDE_DIRS})
target_link_libraries(trimeshdop_demo PRIVATE 
  $<$<STREQUAL:${LIBRARY_TYPE},STATIC>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},BOTH>:pe_static>
  $<$<STREQUAL:${LIBRARY_TYPE},SHARED>:pe_shared>
)