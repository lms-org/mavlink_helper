#ifndef MAVLINK_CSV_LOGGER_H
#define MAVLINK_CSV_LOGGER_H

#include <lms/module.h>

#include <fstream>
#include <tuple>
#include <vector>
#include <unordered_map>

#include <mavlink/CC2016/mavlink.h>
#include <mavlink/lms/data.h>

class MavlinkCsvLogger : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
    
protected:
    //! LogMessageIdentifier tuple (sysid, msgid, compid)
    typedef std::tuple<uint8_t, uint8_t, uint8_t> LogMessageIdentifier;
    
    struct LogMessageIdentifierHash : public std::unary_function<LogMessageIdentifier, std::size_t>
    {
        std::size_t operator()(const LogMessageIdentifier& id) const
        {
            return std::get<0>(id) + std::get<1>(id) + std::get<2>(id);
        }
    };
    
protected:
    void logMessage(const mavlink_message_info_t& msgInfo, const mavlink_message_t& msg);
    std::ofstream& getLogfile(const LogMessageIdentifier& id);
    std::string generateFilename(const LogMessageIdentifier& id);
    void printHeader(std::ofstream& out, uint8_t msgid);
    
    template<typename T>
    void logField(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg);
    
protected:
    //! In data channel (messages received)
    lms::ReadDataChannel<Mavlink::Data> mavlinkChannel;
    
    //! Logfile prefix
    std::string prefix;
    //! LogMessageIdentifier -> logfile mapping
    std::unordered_map<LogMessageIdentifier, std::ofstream, LogMessageIdentifierHash> files;
};

#endif // MAVLINK_CSV_LOGGER_H
