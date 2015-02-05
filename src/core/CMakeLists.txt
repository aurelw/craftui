include_directories (../common/)
include_directories (../utils/)

set (core_source_files elementtype.cpp buttontype.cpp calibsquaretype.cpp slidertype.cpp elementstorage.cpp element.cpp button.cpp slider.cpp openni_interface.cpp openni_interface_connection.cpp colordescriptor.cpp eventgenerator.cpp)

add_library (core ${core_source_files})
target_link_libraries (core ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

