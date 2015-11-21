#ifndef MAVLINK_CSV_LOGGER_H
#define MAVLINK_CSV_LOGGER_H

#include <lms/datamanager.h>
#include <lms/module.h>

class MavlinkCsvLogger : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
};

#endif // MAVLINK_CSV_LOGGER_H
