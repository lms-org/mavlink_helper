#include "car_to_mavlink.h"

bool CarToMavlink::initialize() {
    
    // Datachannels
    car = readChannel<sensor_utils::Car>("CAR");
    inChannel = readChannel<Mavlink::Data>("MAVLINK_IN");
    outChannel = writeChannel<Mavlink::Data>("MAVLINK_OUT");

    lastRcState = false;
    return true;
}

bool CarToMavlink::deinitialize() {
    return true;
}

bool CarToMavlink::cycle() {
    
    checkHeartbeat();
    setControlCommands();
    
    return true;
}


void CarToMavlink::checkHeartbeat()
{
    // Search inChannel for HEARTBEAT message
    for( const auto& msg : *inChannel )
    {
        if( msg.msgid == MAVLINK_MSG_ID_HEARTBEAT )
        {
            // HEARTBEAT found
            
            // check rc state
            auto currentRcState = mavlink_msg_heartbeat_get_remote_control(&msg);
            if( currentRcState != lastRcState )
            {
                if( currentRcState == REMOTE_CONTROL_STATUS_DISCONNECTED )
                {
                    // Lost connection to remote control
                    logger.error("cycle") << "Remote Control connection has been lost!";
                } else if( currentRcState == REMOTE_CONTROL_STATUS_CONNECTED )
                {
                    if( lastRcState == REMOTE_CONTROL_STATUS_DISCONNECTED )
                    {
                        // Re-established connection
                        logger.error("cycle") << "Remote Control connection has been re-established!";
                    } else if( lastRcState == REMOTE_CONTROL_STATUS_ACTIVE )
                    {
                        // Switched off manual mode
                        messaging()->send("RC_STATE_CHANGED", std::to_string(0));
                        logger.warn("cycle") << "Remote Control has been switched to AUTONOMOUS mode!";
                    }
                } else
                {
                    // Switched into manual mode
                    messaging()->send("RC_STATE_CHANGED", std::to_string(1));
                    logger.warn("cycle") << "Remote Control has been switched to MANUAL mode!";
                }
                
                // Update rc state
                lastRcState = currentRcState;
            }
            
            // check drive mode
            // auto currentDriveMode = mavlink_msg_heartbeat_get_drive_mode(&msg);
            // TODO!
        }
    }
}

void CarToMavlink::setControlCommands()
{
    mavlink_message_t msg;
    // TODO: turn signal indicators
    mavlink_msg_control_command_pack(0, 0, &msg, car->targetSpeed(), car->steeringFront(), car->steeringRear(), 0, 0);
    outChannel->add(msg);
}
