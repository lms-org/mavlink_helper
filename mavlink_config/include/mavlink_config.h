#ifndef MAVLINK_CONFIG_H
#define MAVLINK_CONFIG_H

#include <lms/datamanager.h>
#include <lms/module.h>

/**
 * @brief LMS module mavlink_config
 **/
class MavlinkConfig : public lms::Module {
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;
};

#endif // MAVLINK_CONFIG_H
