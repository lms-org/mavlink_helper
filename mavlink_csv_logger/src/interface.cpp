#include "mavlink_csv_logger.h"

extern "C" {
void* getInstance () {
    return new MavlinkCsvLogger();
}
}
