#ifndef CAR_TO_MAVLINK_H
#define CAR_TO_MAVLINK_H

#include <lms/datamanager.h>
#include <lms/module.h>

#include <sensor_utils/car.h>

#include <vector>

#include <mavlink/CC2016/mavlink.h>
#include <mavlink/lms/data.h>

class CarToMavlink : public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
    
protected:
    void parseHeartBeat(const mavlink_message_t &msg);
    void setControlCommands();
    void parseIncomingMessages();
    
protected:
    // Datachannels
    lms::ReadDataChannel<sensor_utils::Car> car;
    lms::ReadDataChannel<Mavlink::Data> inChannel;
    lms::WriteDataChannel<Mavlink::Data> outChannel;
    
    // RC state
    uint8_t lastRcState;
};

#endif // CAR_TO_MAVLINK_H
