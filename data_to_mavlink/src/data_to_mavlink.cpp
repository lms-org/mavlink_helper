#include "data_to_mavlink.h"
#include "phoenix_CC2016_service/phoenix_CC2016_service.h"

bool CarToMavlink::initialize() {
    
    // Datachannels
    car = readChannel<sensor_utils::Car>("CAR");
    outChannel = writeChannel<Mavlink::Data>("MAVLINK_OUT");

    lastRcState = false;
    return true;
}

bool CarToMavlink::deinitialize() {
    return true;
}

bool CarToMavlink::cycle() {
    setControlCommands();
    
    return true;
}



void CarToMavlink::setControlCommands(){
    mavlink_message_t msg;
    // TODO: turn signal indicators
    logger.debug("setControlCommands")<<car->getPrioState().name << ": "<<car->targetSpeed();
    mavlink_msg_control_command_pack(0, 0, &msg, car->targetSpeed(), car->steeringFront(), car->steeringRear(), car->getPrioState().indicatorLeft, car->getPrioState().indicatorRight);
    outChannel->add(msg);
}
