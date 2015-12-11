set(SOURCES
    "src/data_to_mavlink.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/data_to_mavlink.h"
)

include_directories(include)
add_library(data_to_mavlink MODULE ${SOURCES} ${HEADERS})
target_link_libraries(data_to_mavlink PRIVATE lmscore sensor_utils)
