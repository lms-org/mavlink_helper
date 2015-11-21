set(SOURCES
    "src/mavlink_debug.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/mavlink_debug.h"
)

include_directories(include)
add_library(mavlink_debug MODULE ${SOURCES} ${HEADERS})
target_link_libraries(mavlink_debug PRIVATE lmscore)
