"""Loam runtime: the site controller over MQTT, and a simulated site to test it against.

`loam run` is the controller a real site would use. `loam simulate-site` starts a
world, a lidar probe and one process per machine that speak Loam Wire over MQTT,
so the controller can be tested end to end exactly as it would talk to hardware.
Needs `pip install loam[run]` (paho-mqtt) and an MQTT broker such as Mosquitto.
"""
