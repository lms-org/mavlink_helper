set(SOURCES
    "src/mavlink_csv_logger.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/mavlink_csv_logger.h"
)

include_directories(include)
add_library(mavlink_csv_logger MODULE ${SOURCES} ${HEADERS})
target_link_libraries(mavlink_csv_logger PRIVATE lmscore)
