include_directories (../common/)

set (core_source_files elementtype.cpp buttontype.cpp calibsquaretype.cpp slidertype.cpp elementstorage.cpp)

add_library (core ${core_source_files})
target_link_libraries (core ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

