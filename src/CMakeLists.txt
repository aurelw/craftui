add_subdirectory (core)
#add_subdirectory (gui)
add_subdirectory (common)
#add_subdirectory (calib)
add_subdirectory (tools)

if (BUILD_TESTS)
    #    add_subdirectory (tests)
endif (BUILD_TESTS)

if (BUILD_APP)
    #    add_subdirectory (app)
endif (BUILD_APP)

include_directories (./core/)
#include_directories (./gui/)
include_directories (./common/)

