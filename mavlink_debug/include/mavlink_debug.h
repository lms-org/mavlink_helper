#ifndef MAVLINK_DEBUG_H
#define MAVLINK_DEBUG_H

#include <lms/datamanager.h>
#include <lms/module.h>
#include <lms/extra/time.h>

#include <sstream>
#include <map>
#include <tuple>

#include <mavlink/CC2016/mavlink.h>
#include <mavlink/lms/data.h>

class umavlinkudebug : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
protected:
    void printMessage(const mavlink_message_info_t& msgInfo, const mavlink_message_t& msg);
    
    template<typename T>
    void printField(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg);
protected:
    //! Module config
    const lms::ModuleConfig* config;
    //! In data channel (messages received)
    lms::ReadDataChannel<Mavlink::Data> mavlinkChannel;
    //! Message counter
    std::map<std::tuple<uint8_t, uint8_t, uint8_t>, ssize_t> messageCounter;
    //! Timestamp
    lms::extra::PrecisionTime lastTimestamp;
};

#endif // MAVLINK_DEBUG_H
