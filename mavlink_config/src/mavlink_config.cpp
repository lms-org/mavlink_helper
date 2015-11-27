#include <mavlink_config.h>

bool MavlinkConfig::initialize() {
    // Setup datachannels
    inChannel = datamanager()->readChannel< std::vector<mavlink_message_t> >(this, "MAVLINK_IN");
    outChannel = datamanager()->writeChannel< std::vector<mavlink_message_t> >(this, "MAVLINK_OUT");

    // Read system id
    systemID = config().get<size_t>("system_id");

    return true;
}

bool MavlinkConfig::deinitialize() {
    return true;
}

bool MavlinkConfig::cycle() {
    // Request config count
    requestConfigCount();
    return true;
}

void MavlinkConfig::requestConfigCount()
{
    mavlink_config_request_count_t request;
    request.dummy = 0;

    mavlink_message_t msg;
    mavlink_msg_config_request_count_encode(systemID, 0, &msg, &request);
    outChannel->push_back(msg);
}

void MavlinkConfig::requestConfig(size_t config_id)
{

}

void MavlinkConfig::requestParam(size_t config_id, size_t param_id)
{

}

