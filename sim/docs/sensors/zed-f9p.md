# ZED-F9P Model

The ZED-F9P runs standalone on UART1 at 460800 baud. The default profile is HPG
1.51 with UBX protocol 27.50, 5 Hz navigation, GPS, GLONASS, Galileo, BeiDou,
and a separate 1 PPS TIMEPULSE signal.

The UART stream contains exact UBX framing for:

- `UBX-NAV-PVT`, with time, fix flags, geodetic position, NED velocity, and
  accuracy fields
- `UBX-NAV-COV`, with position and velocity covariance in NED
- `UBX-TIM-TP`, paired with the physical TIMEPULSE event

Frames include sync bytes, class, ID, little-endian length and payload, plus
CK_A and CK_B. Arrival includes configured receiver latency, jitter, ten UART
bits per byte, and serialization against other scheduled messages.

PVT and covariance are joined by `iTOW`, then converted to a canonical
launch-centered ENU fix. The ESKF only receives complete pairs. The model keeps
antenna lever arm, correlated random walk, stochastic outages, and delayed
rewind and replay behavior.

The receiver begins prelocked. Above 4 g it continues sending messages but
clears `gnssFixOK`; position updates are rejected. Recovery requires one
continuous second below 4 g. Time validity remains independent, so PPS may stay
valid through boost. RTK, RTCM, SPARTN, and base-station behavior are deferred.

References: [ZED-F9P integration manual](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)
and [HPG 1.51 interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.51_InterfaceDescription_UBXDOC-963802114-13124.pdf).
