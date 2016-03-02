#include "mavlink_debug.h"

#include <type_traits>

static mavlink_message_info_t msgInfos[256] = MAVLINK_MESSAGE_INFO;

bool umavlinkudebug::initialize() {
    // Setup datachannels
    mavlinkChannel = readChannel<Mavlink::Data>("CHANNEL");
    
    // Reset timestamp
    lastTimestamp = lms::Time::now();
    
    return true;
}

bool umavlinkudebug::deinitialize() {
    return true;
}

bool umavlinkudebug::cycle() {
    
    // Measure time-delta
    auto timeDelta = lastTimestamp.since().toFloat();
    lastTimestamp = lms::Time::now();
    
    messageCounter.clear();
    for( const auto& msg : *mavlinkChannel )
    {
        // Print message
        printMessage(msgInfos[msg.msgid], msg);
        
        // Count message
        auto key = std::make_tuple(msg.sysid, msg.msgid, msg.compid);
        messageCounter[key]++;
    }
    
    // Print message counter
    for(const auto& ctr : messageCounter)
    {
        uint8_t sid, mid, cid;
        std::tie(sid, mid, cid) = ctr.first;
        
        logger.info()   << "(" << size_t(sid) << ") "
                        << msgInfos[mid].name
                        << "[" << size_t(cid) << "]: "
                        << ctr.second << " msgs "
                        << "[ " << ctr.second / timeDelta << " msgs/sec ]";
    }
    
    return true;
}

void umavlinkudebug::printMessage(const mavlink_message_info_t& msgInfo, const mavlink_message_t& msg)
{
    std::stringstream ss;
    
    ss << "(" << size_t(msg.sysid) << ") " << msgInfo.name << "[" << size_t(msg.compid) << "] ";
    for(size_t i = 0; i < msgInfo.num_fields; i++)
    {
        const mavlink_field_info_t& field = msgInfo.fields[i];
        switch(field.type)
        {
            case MAVLINK_TYPE_CHAR:
                printField<char>(ss, field, msg);
                break;
            case MAVLINK_TYPE_UINT8_T:
                printField<uint8_t>(ss, field, msg);
                break;
            case MAVLINK_TYPE_INT8_T:
                printField<int8_t>(ss, field, msg);
                break;
            case MAVLINK_TYPE_UINT16_T:
                printField<uint16_t>(ss, field, msg);
                break;
            case MAVLINK_TYPE_INT16_T:
                printField<int16_t>(ss, field, msg);
                break;
            case MAVLINK_TYPE_UINT32_T:
                printField<uint32_t>(ss, field, msg);
                break;
            case MAVLINK_TYPE_INT32_T:
                printField<int32_t>(ss, field, msg);
                break;
            case MAVLINK_TYPE_UINT64_T:
                printField<uint64_t>(ss, field, msg);
                break;
            case MAVLINK_TYPE_INT64_T:
                printField<int64_t>(ss, field, msg);
                break;
            case MAVLINK_TYPE_FLOAT:
                printField<float>(ss, field, msg);
                break;
            case MAVLINK_TYPE_DOUBLE:
                printField<double>(ss, field, msg);
                break;
        }
        
        if( i < (msgInfo.num_fields-1))
        {
            ss << " | ";
        }
    }
    logger.debug() << ss.str();
}

template<typename T>
void umavlinkudebug::printField(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg)
{
    ss << field.name << ": ";
    
    const T* ptr = reinterpret_cast<const T*>( reinterpret_cast<const uint8_t*>(&msg.payload64) + field.structure_offset );
    if( field.array_length == 0 )
    {
        if( std::is_same<T, int8_t>::value )
        {
            // Force proper display format
            ss << static_cast<ssize_t>(*ptr);
        }
        else if( std::is_same<T, uint8_t>::value )
        {
            // Force proper display format
            ss << static_cast<size_t>(*ptr);
        }
        else
        {
            ss << *ptr;
        }
    }
    else
    {
        // Array
        if( std::is_same<T, char>::value )
        {
            // Char array aka string
            ss << ptr;
        }
        else
        {
            // non-char array
            ss << "[ ";
            for( size_t i = 0; i < field.array_length; i++ )
            {
                ss << *(ptr+i);
                if( i < (field.array_length-1))
                {
                    ss << ", ";
                }
            }
            ss << " ]";
        }
    }
}

template void umavlinkudebug::printField<char>(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void umavlinkudebug::printField<int8_t>(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void umavlinkudebug::printField<uint8_t>(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void umavlinkudebug::printField<int16_t>(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void umavlinkudebug::printField<uint16_t>(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void umavlinkudebug::printField<int32_t>(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void umavlinkudebug::printField<uint32_t>(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void umavlinkudebug::printField<int64_t>(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void umavlinkudebug::printField<uint64_t>(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void umavlinkudebug::printField<float>(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void umavlinkudebug::printField<double>(std::stringstream& ss, const mavlink_field_info_t& field, const mavlink_message_t& msg);

