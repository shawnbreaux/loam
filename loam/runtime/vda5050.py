"""A VDA 5050 3.0 mobile robot as a Loam machine.

`loam vda5050` plays the fleet-control side of VDA 5050 for one robot, on
`<interface>/v3/<manufacturer>/<serialNumber>/{order,instantActions,state,connection}`.
Positions are metres and radians on the map whose `mapId` is the site name,
with the same axes as the site grid.

* A Loam order becomes a VDA order: one node per path point, edges with the
  order's `maximumSpeed`, and the work as a `loam.<op>` action (blocking type
  NONE) on the work edge, its parameters carrying the Loam order id, op,
  window and limits, so a robot's own tool control can run it.
* Pause → `startPause`, resume → `stopPause`, e-stop → `cancelOrder` plus
  `startPause`. VDA 5050 has no remote e-stop; a robot's own e-stop shows in
  `safetyState.activeEmergencyStop` and holds the site machine.
* The pass is done when the robot reports the last node reached and no node,
  edge or action left.

openTCS (MIT) is also a VDA 5050 fleet control. A robot runs under one fleet
control at a time, so a robot used here is taken out of openTCS for the job.
"""
from __future__ import annotations

import time
from datetime import datetime, timezone

from ..checker import Model
from ..units import Quantity
from .bridge import run_bridge
from .bus import Bus, tolerant

VERSION = "3.0.0"
STATE_STALE_S = 35.0                     # robots publish state on change and at least every 30 s
ORDER_ACK_S = 10.0                       # a robot whose state does not show a new order by then has not taken it


def iso(t: float | None = None) -> str:
    """A time (now, by default) as VDA 5050 writes it: UTC, to the millisecond, ending in Z."""
    d = datetime.fromtimestamp(time.time() if t is None else t, timezone.utc)
    return d.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


class Robot:
    """The fleet-control side of VDA 5050 for one robot, as a bridge adapter."""

    def __init__(self, broker: str, manufacturer: str, serial: str, map_id: str, unit_m: float,
                 interface: str = "uagv", client: str | None = None):
        """Connect to the broker and listen to the robot's state and connection topics. `map_id` is the site name;
        `unit_m` is the length of one site unit in metres (VDA 5050 is metric).
        """
        self.base = f"{interface}/v3/{manufacturer}/{serial}"
        self.manufacturer, self.serial, self.map_id, self.u = manufacturer, serial, map_id, unit_m
        self.bus = Bus(broker, client or f"loam-vda-{serial}-{time.time():.3f}")
        self.bus.start([f"{self.base}/state", f"{self.base}/connection"])
        self.header = 0
        self.pose: tuple[float, float, float] | None = None
        self.state: dict = {}
        self.state_at = 0.0
        self.connection = "OFFLINE"
        self.order_id: str | None = None
        self.last_seq = 0
        self.paused_sent: bool | None = None
        self.ordered_at = 0.0

    def _msg(self, **body) -> dict:
        """A VDA 5050 message: the common header (a counter, the time, version, manufacturer and serial number)
        followed by `body`.
        """
        self.header += 1
        return {"headerId": self.header, "timestamp": iso(), "version": VERSION,
                "manufacturer": self.manufacturer, "serialNumber": self.serial, **body}

    def _actions(self, *types: str, **params) -> None:
        """Send instant actions (startPause, stopPause, cancelOrder) to the robot."""
        acts = [{"actionId": f"{a}-{self.header + 1}-{i}", "actionType": a, "blockingType": "NONE",
                 **({"actionParameters": [{"key": k, "value": v} for k, v in params.items()]} if params else {})}
                for i, a in enumerate(types)]
        self.bus.pub(f"{self.base}/instantActions", self._msg(actions=acts), qos=1)

    # -- adapter -------------------------------------------------------------------------
    def poll(self) -> None:
        """Read the robot's state and connection messages that have arrived. Positions on another map are ignored."""
        for topic, msg in self.bus.drain(0.0):
            with tolerant(topic):
                if topic.endswith("/connection"):
                    self.connection = msg.get("connectionState", "OFFLINE")
                elif topic.endswith("/state"):
                    self.state, self.state_at = msg, time.time()
                    p = msg.get("mobileRobotPosition")
                    if p and p.get("mapId") == self.map_id:
                        self.pose = (p["x"] / self.u, p["y"] / self.u, float(p["theta"]))

    @property
    def fix_ok(self) -> bool:
        """Does the robot know where it is on the map?"""
        return bool((self.state.get("mobileRobotPosition") or {}).get("localized"))

    @property
    def fix_name(self) -> str:
        """The localization in words, for the state message."""
        return "localized" if self.fix_ok else "not localized"

    @property
    def fault(self) -> str | None:
        """Why the robot cannot work, or None. In order: it is offline or silent; its own emergency stop is active; it
        is not in an automatic mode; it reports a critical or fatal error.
        """
        st = self.state
        if self.connection != "ONLINE" or time.time() - self.state_at > STATE_STALE_S:
            return "vehicleLinkLost"
        if (st.get("safetyState") or {}).get("activeEmergencyStop", "NONE") != "NONE":
            return "vehicleEstop"
        if st.get("operatingMode") not in ("AUTOMATIC", "SEMIAUTOMATIC"):
            return "vehicleNotAutomatic"
        if any(e.get("errorLevel") in ("CRITICAL", "FATAL") for e in st.get("errors", [])):
            return "vehicleError"
        return None

    @property
    def order_error(self) -> str | None:
        """`vehicleOrderRejected` when the robot's state still does not show the order 10 s after it was sent, else
        None.
        """
        if self.order_id is not None and self.state.get("orderId") != self.order_id \
                and time.time() - self.ordered_at > ORDER_ACK_S:
            return "vehicleOrderRejected"            # it never took the order: do not wait on it forever
        return None

    def start(self, path: list[tuple[float, float]], order: dict, max_speed_ms: float | None) -> None:
        """Send the order. Nodes are numbered 0, 2, 4, ... and edges take the odd numbers between, so the last node is
        2 x (points - 1).
        """
        self.bus.pub(f"{self.base}/order", vda_order(self._msg, order, path, self.map_id, self.u, max_speed_ms),
                     qos=1)
        self.order_id, self.last_seq = order["orderId"], 2 * (len(path) - 1)
        self.paused_sent, self.ordered_at = None, time.time()

    def pause(self, on: bool) -> None:
        """Pause or release the robot. Sent once per change, since the broker delivers it reliably (QoS 1)."""
        if on != self.paused_sent and (self.order_id is not None or on):
            self._actions("startPause" if on else "stopPause")
            self.paused_sent = on

    def estop(self) -> None:
        """Site e-stop: cancel the order and pause. The robot's own safety system does the actual stopping."""
        self._actions("cancelOrder", "startPause")
        self.paused_sent = True

    def fraction(self) -> float:
        """How far through the order, 0 to 1, by the last node reached."""
        if self.state.get("orderId") != self.order_id or not self.last_seq:
            return 0.0
        return min(1.0, self.state.get("lastNodeSequenceId", 0) / self.last_seq)

    def done(self) -> bool:
        """Is the order complete: last node reached, nothing left to drive, every action finished or failed?"""
        st = self.state
        return (self.order_id is not None and st.get("orderId") == self.order_id
                and st.get("lastNodeSequenceId") == self.last_seq and not st.get("nodeStates")
                and not st.get("edgeStates")
                and all(a.get("actionStatus") in ("FINISHED", "FAILED") for a in st.get("actionStates", [])))

    def close(self) -> None:
        """Disconnect from the broker."""
        self.bus.stop()


def vda_order(msg, order: dict, path: list[tuple[float, float]], map_id: str, u: float,
              max_speed_ms: float | None) -> dict:
    """A Loam order and its path (site units) as a VDA 5050 order (metres). Node 0 is where the robot is;
    the work is an action on the first edge that crosses the work window."""
    nodes = [{"nodeId": f"{order['orderId']}-n{i}", "sequenceId": 2 * i, "released": True,
              "nodePosition": {"x": round(x * u, 4), "y": round(y * u, 4), "mapId": map_id}, "actions": []}
             for i, (x, y) in enumerate(path)]
    params: list[dict] = [{"key": "loamOrderId", "value": order["orderId"]}, {"key": "op", "value": order["op"]},
                          {"key": "limits", "value": order.get("limits") or {}}]
    if order.get("window"):
        params.append({"key": "window", "value": [[round(x * u, 4), round(y * u, 4)] for x, y in order["window"]]})
    work = {"actionId": f"{order['orderId']}-work", "actionType": f"loam.{order['op']}", "blockingType": "NONE",
            "actionParameters": params}
    edges = []
    for i in range(len(path) - 1):
        e: dict = {"edgeId": f"{order['orderId']}-e{i}", "sequenceId": 2 * i + 1, "released": True,
                   # The path is: where the robot is, the start of the work pass, its end, then the haul. So the work
                   # pass is the second edge (or the only one).
                   "actions": [work] if i == 1 or (i == 0 and len(path) == 2) else []}
        if max_speed_ms:
            e["maximumSpeed"] = max_speed_ms
        edges.append(e)
    if not edges:
        nodes[0]["actions"] = [work]
    return msg(orderId=order["orderId"], orderUpdateId=0, nodes=nodes, edges=edges)


def run_vda5050(model: Model, name: str, broker: str, manufacturer: str, serial: str,
                interface: str = "uagv", sim_world: bool = False, max_wall_s: float | None = None):
    """Bridge one VDA 5050 robot as Loam machine `name` (see `bridge.run_bridge`)."""
    u = Quantity(1, model.site.get("units", "ft")).si
    robot = Robot(broker, manufacturer, serial, model.site["name"], u, interface)
    return run_bridge(model, name, broker, robot, sim_world, max_wall_s)
