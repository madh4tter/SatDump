#include "imu_handler.h"
#include "core/plugin.h"

#include "bno055_handler.h"

namespace imu
{
    std::vector<ImuHandlerOption> getImuHandlerOptions()
    {
        std::vector<ImuHandlerOption> imuOptions;

        imuOptions.push_back({"bno055", []()
                               { return std::make_shared<Bno055Handler>(); }});

        satdump::eventBus->fire_event<RequestImuHandlerOptionsEvent>({imuOptions});

        return imuOptions;
    }
}
