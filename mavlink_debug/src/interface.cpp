#include "mavlink_debug.h"

extern "C" {
void* getInstance () {
    return new umavlinkudebug();
}
}
