#ifndef MAVLINK_CONFIG_H
#define MAVLINK_CONFIG_H

#include <lms/module.h>

#include <vector>

#include <mavlink/CC2016/mavlink.h>
#include <mavlink/lms/data.h>

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
    lms::WriteDataChannel<Mavlink::Data> outChannel;
    //! In data channel
    lms::ReadDataChannel<Mavlink::Data> inChannel;

    //! Mavlink System ID
    uint8_t systemID;
};

#endif // MAVLINK_CONFIG_H
