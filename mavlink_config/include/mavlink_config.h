#ifndef MAVLINK_CONFIG_H
#define MAVLINK_CONFIG_H

#include <lms/datamanager.h>
#include <lms/module.h>

#include <vector>
#include <mavlink/CC2016/mavlink.h>

/**
 * @brief LMS module mavlink_config
 **/
class MavlinkConfig : public lms::Module {
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;

protected:
    void requestConfigCount();
    void requestConfig(size_t config_id);
    void requestParam(size_t config_id, size_t param_id);

protected:
    //! Out data channel
    lms::WriteDataChannel< std::vector<mavlink_message_t> > outChannel;
    //! In data channel
    lms::ReadDataChannel< std::vector<mavlink_message_t> > inChannel;

    //! Mavlink System ID
    uint8_t systemID;
};

#endif // MAVLINK_CONFIG_H
