set(SOURCES
    "src/car_to_mavlink.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/car_to_mavlink.h"
)

include_directories(include)
add_library(car_to_mavlink MODULE ${SOURCES} ${HEADERS})
target_link_libraries(car_to_mavlink PRIVATE lmscore)
