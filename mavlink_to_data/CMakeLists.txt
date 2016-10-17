set(SOURCES
    "src/mavlink_to_data.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/mavlink_to_data.h"
    "include/sensors.h"
)

include_directories(include)
add_library(mavlink_to_data MODULE ${SOURCES} ${HEADERS})
target_link_libraries(mavlink_to_data PRIVATE lmscore lms_sensor_utils street_environment phoenix_CC2016_service timestamp_interpolator_service)
