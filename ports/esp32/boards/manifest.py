freeze("$(PORT_DIR)/modules")
#freeze("$(PORT_DIR)/modules/seeker_lib")
include("$(MPY_DIR)/extmod/asyncio")
#include("$(MPY_DIR)/extmod/seeker_lib")

# Useful networking-related packages.
require("bundle-networking")

# Require some micropython-lib modules.
require("aioespnow")
require("dht")
require("ds18x20")
require("neopixel")
require("onewire")
require("umqtt.robust")
require("umqtt.simple")
require("upysh")
