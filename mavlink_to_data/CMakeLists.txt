set(SOURCES
    "src/mavlink_to_data.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/mavlink_to_data.h"
)

include_directories(include)
add_library(mavlink_to_data MODULE ${SOURCES} ${HEADERS})
target_link_libraries(mavlink_to_data PRIVATE lmscore)
