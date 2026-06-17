#include "gps_handler.h"
#include "core/plugin.h"

#include "gpsd_handler.h"

namespace gps
{
    std::vector<GpsHandlerOption> getGpsHandlerOptions()
    {
        std::vector<GpsHandlerOption> gpsOptions;

        gpsOptions.push_back({"gpsd", []()
                               { return std::make_shared<GpsdHandler>(); }});

        satdump::eventBus->fire_event<RequestGpsHandlerOptionsEvent>({gpsOptions});

        return gpsOptions;
    }
}
