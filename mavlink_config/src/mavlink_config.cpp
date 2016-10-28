#include <mavlink_config.h>

bool MavlinkConfig::initialize() {
    // Setup datachannels
    inChannel = readChannel<Mavlink::Data>("MAVLINK_IN");
    outChannel = writeChannel<Mavlink::Data>("MAVLINK_OUT");

    // Read system id
    systemID = config().get<int>("system_id");

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
    outChannel->add(msg);
}

void MavlinkConfig::requestConfig(size_t config_id)
{
    mavlink_config_request_t request;
    request.config_id = config_id;

    mavlink_message_t msg;
    mavlink_msg_config_request_encode(systemID, 0, &msg, &request);
    outChannel->add(msg);
}

void MavlinkConfig::requestParam(size_t config_id, size_t param_id)
{
    mavlink_config_request_params_t request;
    request.config_id = config_id;
    request.param_id = param_id;

    mavlink_message_t msg;
    mavlink_msg_config_request_params_encode(systemID, 0, &msg, &request);
    outChannel->add(msg);
}

