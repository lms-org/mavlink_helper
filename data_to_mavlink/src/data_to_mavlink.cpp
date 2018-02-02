#include "data_to_mavlink.h"
#include "phoenix_CC2016_service/phoenix_CC2016_service.h"

bool CarToMavlink::initialize() {
    
    // Datachannels
    car = readChannel<street_environment::CarCommand>("CAR");
    outChannel = writeChannel<Mavlink::Data>("MAVLINK_OUT");

    lastRcState = false;
    return true;
}

bool CarToMavlink::deinitialize() {
    return true;
}

bool CarToMavlink::cycle() {
    setControlCommands();
    setDriveMode();
    
    return true;
}



void CarToMavlink::setControlCommands(){
    mavlink_message_t msg;
    // TODO: turn signal indicators
    logger.debug("setControlCommands")<<car->getPrioState().name << ": "<<car->targetSpeed();
    mavlink_msg_control_command_pack(0, 0, &msg, car->targetSpeed(), car->steeringFront(), car->steeringRear(), car->getPrioState().indicatorLeft, car->getPrioState().indicatorRight);
    outChannel->add(msg);
}


void CarToMavlink::setDriveMode(){
    if(street_environment::CarCommand::StateType::PARKING_FINISHED == car->getPrioState().state ||
        street_environment::CarCommand::StateType::PARKING_START == car->getPrioState().state )
    {
        mavlink_message_t msg;
        // todo: put this in config file
        const uint8_t drive_mode_config_set = 1;
        const uint8_t drive_mode_param_id = 3;
        int32_t mode;

        switch(car->getPrioState().state)
        {
        case street_environment::CarCommand::StateType::PARKING_FINISHED:
            mode = DRIVE_MODE::DRIVE_MODE_TRACK; // switch back to FOH
            break;
        case street_environment::CarCommand::StateType::PARKING_START:
            mode = DRIVE_MODE::DRIVE_MODE_PARKING; // start parking
            break;
        }

        mavlink_msg_config_param_set_int_pack(0, 0, &msg, drive_mode_config_set, drive_mode_param_id, mode);
        outChannel->add(msg);

    }



}
