# Mesh Firmware
Currently UNDER DEVELOPMENT.

This is a wireless mesh network system which integrates smart home/industry sensing, tracking and security into a single network.

The wireless mesh network is a proprietary, decentralized flood type network. The network consists of three distinct node types: routers (R), gateways (GT) and low-power end devices (LP).
Routers retransmit received data to form the backbone of the network.
Gateways are similar to routers and forward mesh network data to a server/cloud.
End devices are low-power nodes that sleep for most of the time, periodically transmitting data.

This project involves the design of several hardware devices (https://github.com/edward62740/Mesh-Hardware.git), as well as firmware. It is important to note that each hardware device may be compatible with one or more firmware types.


# Compatibility Matrix
y-axis is the HARDWARE DEVICE, x-axis is the FIRMWARE TYPE. if compatible, node type is shown.
It is recommended to use the highlighted combination where possible.
|            | Environment | Motion | Airquality | VOC | Proximity | Gateway | Router | Controller |
|------------|-------------|--------|------------|-----|-----------|---------|--------|------------|
| GPSN       |     LP      |   LP   |     LP     |     |           |         |        |            |
| VOCSN      |             |        |            |  LP |           |         |        |            |
| Gateway    |             |        |            |     |           |    GT   |        |            |
| Router     |             |        |            |     |           |         |    R   |            |
| Controller |             |        |            |     |           |         |        |     R      |

# Completion % Sept 2020
|            | Environment | Motion | Airquality | VOC | Proximity | Gateway | Router | Controller |
|------------|-------------|--------|------------|-----|-----------|---------|--------|------------|
|            |     100     |   85   |    DEPR    | 15  |    15     |   95    |   90   |     75     |


Released under the GPL-3.0 License