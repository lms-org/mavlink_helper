set(SOURCES
    "src/mavlink_config.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/mavlink_config.h"
)

include_directories(include)
add_library(mavlink_config MODULE ${SOURCES} ${HEADERS})
target_link_libraries(mavlink_config PRIVATE lmscore)
