#ifndef CAR_TO_MAVLINK_H
#define CAR_TO_MAVLINK_H

#include <lms/datamanager.h>
#include <lms/module.h>

#include <sensor_utils/car.h>

#include <vector>

#include <mavlink/CC2016/mavlink.h>

class CarToMavlink : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
    
protected:
    void checkHeartbeat();
    void setControlCommands();
    
protected:
    // Datachannels
    lms::ReadDataChannel<sensor_utils::Car> car;
    lms::ReadDataChannel<std::vector<mavlink_message_t> > inChannel;
    lms::WriteDataChannel<std::vector<mavlink_message_t> > outChannel;
    
    // RC state
    uint8_t lastRcState;
};

#endif // CAR_TO_MAVLINK_H
