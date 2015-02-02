include_directories (../common/)
include_directories (../core/)
include_directories (../gui/)

set (calib_source_files dynamicthreshcalib.cpp)
add_library (calib ${calib_source_files})
target_link_libraries(calib core)

