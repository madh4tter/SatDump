#include "imu_handler.h"
#include "core/plugin.h"

#include "bno055_handler.h"
#include "bno085_handler.h"

namespace imu
{
    std::vector<ImuHandlerOption> getImuHandlerOptions()
    {
        std::vector<ImuHandlerOption> imuOptions;

        imuOptions.push_back({"bno055", []()
                               { return std::make_shared<Bno055Handler>(); }});

        imuOptions.push_back({"bno085", []()
                               { return std::make_shared<Bno085Handler>(); }});

        satdump::eventBus->fire_event<RequestImuHandlerOptionsEvent>({imuOptions});

        return imuOptions;
    }
}