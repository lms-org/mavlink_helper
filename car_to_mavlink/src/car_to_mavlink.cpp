#include "car_to_mavlink.h"
#include "phoenix_CC2016_service/phoenix_CC2016_service.h"

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
    
    parseIncomingMessages();
    setControlCommands();
    
    return true;
}

void CarToMavlink::parseIncomingMessages(){

    for( const auto& msg : *inChannel ){
        if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT){
            parseHeartBeat(msg);
        }
    }
}


void CarToMavlink::parseHeartBeat(const mavlink_message_t &msg)
{
    // Search inChannel for HEARTBEAT message
        if( msg.msgid == MAVLINK_MSG_ID_HEARTBEAT )
        {
            // HEARTBEAT found
            lms::ServiceHandle<phoenix_CC2016_service::Phoenix_CC2016Service> service = getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE");
            
            //get rc state
            int currentRcState = mavlink_msg_heartbeat_get_remote_control(&msg);
            phoenix_CC2016_service::RemoteControlState rcState = phoenix_CC2016_service::RemoteControlState::IDLE;

            if( currentRcState == REMOTE_CONTROL_STATUS_DISCONNECTED ){
                rcState =phoenix_CC2016_service::RemoteControlState::DISCONNECTED;
            } else if( currentRcState == REMOTE_CONTROL_STATUS_CONNECTED ){
                rcState =phoenix_CC2016_service::RemoteControlState::IDLE;
            }else if(currentRcState == REMOTE_CONTROL_STATUS_ACTIVE){
                rcState =phoenix_CC2016_service::RemoteControlState::ACTIVE;
            }else{
                logger.error("parseHeartBeat")<<"invalid rc-state: "<<currentRcState;
            }
            //TODO get values
            service->update(rcState,phoenix_CC2016_service::CCDriveMode::IDLE,8);
        }

}

void CarToMavlink::setControlCommands(){
    mavlink_message_t msg;
    // TODO: turn signal indicators
    mavlink_msg_control_command_pack(0, 0, &msg, car->targetSpeed(), car->steeringFront(), car->steeringRear(), 0, 0);
    outChannel->add(msg);
}
