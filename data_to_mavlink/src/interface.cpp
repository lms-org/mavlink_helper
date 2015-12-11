#include "data_to_mavlink.h"

extern "C" {
void* getInstance () {
    return new CarToMavlink();
}
}
