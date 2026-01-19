# topic_watchdog

A modular ROS 2 topic watchdog node designed for distributed safety functions.

Tested on Jazzy and Humble.

## Overview

This package monitors specified ROS 2 topics for activity. If a topic fails to send data within a defined threshold, the node publishes a specific error code.

## Purpose

The primary goal is to offload monitoring tasks from resource-constrained hardware. By running this watchdog on a powerful host (e.g., Raspberry Pi 5), an ESP32 or other microcontrollers only need to subscribe to a lightweight `Int32` heartbeat topic instead of data-heavy streams like `sensor_msgs/msg/LaserScan`. This significantly reduces network load and CPU usage on the microcontroller side.

## Parameters

| Parameter | Type | Default Value | Description |
| --- | --- | --- | --- |
| `input_topic` | string | "chatter" | The ROS 2 topic to be monitored. |
| `threshold_ms` | int | 5000 | Maximum allowed time (in milliseconds) between two messages before a timeout is declared. |
| `timer_interval_ms` | int | 100 | Frequency at which the watchdog checks the status. |
| `error_code` | int | 1 | The integer value sent to the heartbeat topic when a timeout occurs. |
| `enable_debug_msg` | bool | false | If true, publishes additional human-readable status strings to a debug topic. |

## Launch

The node is designed to be launched dynamically based on a YAML configuration.

```bash
ros2 launch topic_watchdog watchdog.launch.py

```

The `watchdog.launch.py` script parses the `watchdog_params.yaml` file and automatically instantiates a separate node for every configuration block defined in the file. This allows for monitoring multiple sensors without modifying the source code.

Hier ist die ergänzte und verfeinerte Version für den unteren Teil deiner README. Ich habe die Liste der geplanten Features etwas technischer formuliert, damit sie für andere Entwickler auf GitHub ansprechender wirkt.

### Manual Execution

You can also run a single instance of the watchdog directly via the CLI:

```bash
ros2 run topic_watchdog watchdog_node --ros-args -p input_topic:=cmd_vel -p error_code:=50

```

## Planned Features

* [ ] **Latency and Jitter Measurement:** Monitoring the consistency of message frequencies.
* [ ] **Multi-Topic Support:** Allowing a single node instance to monitor multiple related topics.
* [ ] **Dynamic Publisher Configuration:** Support for multiple output topics per node.
* [ ] **Expanded Diagnostics:** Integration with the standard `diagnostic_msgs` for better ROS 2 ecosystem compatibility.
