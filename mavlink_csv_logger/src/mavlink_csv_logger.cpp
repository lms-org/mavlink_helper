#include "mavlink_csv_logger.h"

#include <lms/extra/time.h>

static mavlink_message_info_t msgInfos[256] = MAVLINK_MESSAGE_INFO;

bool MavlinkCsvLogger::initialize() {
    // Set log file prefix correctly
    prefix = logDir("mavlink_csv");
    
    logger.debug("init") << "Log prefix: " << prefix;
    
    // Setup datachannels
    auto channel = config().get<std::string>("channel");
    mavlinkChannel = readChannel<Mavlink::Data>(channel);
    
    return true;
}

bool MavlinkCsvLogger::deinitialize() {
    for(auto& f : files)
    {
        // Close all opened log files
        f.second.close();
    }
    
    return true;
}

bool MavlinkCsvLogger::cycle() {
    for( const auto& msg : *mavlinkChannel )
    {
        // Log message
        logMessage(msgInfos[msg.msgid], msg);
    }
    return true;
}

void MavlinkCsvLogger::logMessage(const mavlink_message_info_t& msgInfo, const mavlink_message_t& msg)
{
    auto id = std::make_tuple(uint8_t(msg.sysid), uint8_t(msg.msgid), uint8_t(msg.compid));
    auto& out = getLogfile(id);
    
    // Write new line to csv
    for(size_t i = 0; i < msgInfo.num_fields; i++)
    {
        const mavlink_field_info_t& field = msgInfo.fields[i];
        switch(field.type)
        {
            case MAVLINK_TYPE_CHAR:
                logField<char>(out, field, msg);
                break;
            case MAVLINK_TYPE_UINT8_T:
                logField<uint8_t>(out, field, msg);
                break;
            case MAVLINK_TYPE_INT8_T:
                logField<int8_t>(out, field, msg);
                break;
            case MAVLINK_TYPE_UINT16_T:
                logField<uint16_t>(out, field, msg);
                break;
            case MAVLINK_TYPE_INT16_T:
                logField<int16_t>(out, field, msg);
                break;
            case MAVLINK_TYPE_UINT32_T:
                logField<uint32_t>(out, field, msg);
                break;
            case MAVLINK_TYPE_INT32_T:
                logField<int32_t>(out, field, msg);
                break;
            case MAVLINK_TYPE_UINT64_T:
                logField<uint64_t>(out, field, msg);
                break;
            case MAVLINK_TYPE_INT64_T:
                logField<int64_t>(out, field, msg);
                break;
            case MAVLINK_TYPE_FLOAT:
                logField<float>(out, field, msg);
                break;
            case MAVLINK_TYPE_DOUBLE:
                logField<double>(out, field, msg);
                break;
        }
        
        if( i < (msgInfo.num_fields-1))
        {
            out << ",";
        }
    }
    out << std::endl;
}


std::string MavlinkCsvLogger::generateFilename(const LogMessageIdentifier& id)
{
    uint8_t sysid, msgid, compid;
    std::tie(sysid, msgid, compid) = id;
    
    std::ostringstream filename;
    filename << prefix << size_t(sysid) << "_" << msgInfos[msgid].name << "_" << size_t(compid) << ".csv";
    return filename.str();
}

std::ofstream& MavlinkCsvLogger::getLogfile(const LogMessageIdentifier& id)
{
    auto file = files.find(id);
    if(file == files.end())
    {
        // Create new log file
        std::string filename = generateFilename(id);
        files[id].open(filename);
        if(!files[id])
        {
            logger.error() << "Unable to open output file " << filename << ": " << strerror(errno);
        }
        logger.warn() << "Opening logfile " << filename;
        
        // Print header fields
        printHeader(files[id], std::get<1>(id));
    }
    
    return files[id];
}

void MavlinkCsvLogger::printHeader(std::ofstream& out, uint8_t msgid)
{
    const mavlink_message_info_t& msgInfo = msgInfos[ msgid ];
    for(size_t i = 0; i < msgInfo.num_fields; i++)
    {
        const mavlink_field_info_t& field = msgInfo.fields[i];
        if( field.array_length == 0 || field.type == MAVLINK_TYPE_CHAR )
        {
            // Single-element or char array (string)
            out << msgInfo.fields[i].name;
        }
        else
        {
            for( size_t j = 0; j < field.array_length; j++ )
            {
                out << msgInfo.fields[i].name << "[" << j << "]";
                if( j < (field.array_length-1))
                {
                    out << ",";
                }
            }
        }
        
        if( i < (msgInfo.num_fields-1))
        {
            out << ",";
        }
    }
    out << std::endl;
}


template<typename T>
void MavlinkCsvLogger::logField(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg)
{
    const T* ptr = reinterpret_cast<const T*>( reinterpret_cast<const uint8_t*>(&msg.payload64) + field.structure_offset );
    if( field.array_length == 0 )
    {
        if( std::is_same<T, int8_t>::value )
        {
            // Force proper display format
            out << static_cast<ssize_t>(*ptr);
        }
        else if( std::is_same<T, uint8_t>::value )
        {
            // Force proper display format
            out << static_cast<size_t>(*ptr);
        }
        else
        {
            out << *ptr;
        }
    }
    else
    {
        // Array
        if( std::is_same<T, char>::value )
        {
            // Char array aka string
            out << ptr;
        }
        else
        {
            // non-char array
            for( size_t i = 0; i < field.array_length; i++ )
            {
                out << *(ptr+i);
                if( i < (field.array_length-1))
                {
                    out << ",";
                }
            }
        }
    }
}

template void MavlinkCsvLogger::logField<char>(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void MavlinkCsvLogger::logField<int8_t>(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void MavlinkCsvLogger::logField<uint8_t>(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void MavlinkCsvLogger::logField<int16_t>(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void MavlinkCsvLogger::logField<uint16_t>(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void MavlinkCsvLogger::logField<int32_t>(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void MavlinkCsvLogger::logField<uint32_t>(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void MavlinkCsvLogger::logField<int64_t>(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void MavlinkCsvLogger::logField<uint64_t>(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void MavlinkCsvLogger::logField<float>(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg);
template void MavlinkCsvLogger::logField<double>(std::ofstream& out, const mavlink_field_info_t& field, const mavlink_message_t& msg);
