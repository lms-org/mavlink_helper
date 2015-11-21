#ifndef CAR_TO_MAVLINK_H
#define CAR_TO_MAVLINK_H

#include <lms/datamanager.h>
#include <lms/module.h>

class CarToMavlink : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
};

#endif // CAR_TO_MAVLINK_H
