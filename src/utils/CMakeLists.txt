include_directories (../common/)

set (utils_source_files craftui_mathutils.cpp craftui_fileutils.cpp)

add_library (utils ${utils_source_files})
target_link_libraries (utils ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

