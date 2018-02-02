#ifndef DATA_TO_MAVLINK_H
#define DATA_TO_MAVLINK_H

#include <lms/module.h>

#include <street_environment/car.h>
#include <mavlink/CC2016/mavlink.h>
#include <mavlink/lms/data.h>
class CarToMavlink : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
    
protected:
    void setControlCommands();
    void setDriveMode();
    
protected:
    // Datachannels
    lms::WriteDataChannel<Mavlink::Data> outChannel;
    lms::ReadDataChannel<street_environment::CarCommand> car;

    
    // RC state
    uint8_t lastRcState;
};

#endif // DATA_TO_MAVLINK_H
