# Lot 10 lidar and vision simulation

Run the separate simulation with `bash drive_lot10.sh` from this folder. It uses ROS domain 88 by default. The original simulation uses domain 87. On Ubuntu with ROS 2 Jazzy and the Gazebo bridge installed, install `python3-opencv` and `python3-numpy` if needed.

The patrol publishes `/cmd_vel_requested`. The `/lot10_obstacle_guard` node reads the Gazebo lidar on `/scan` and camera on `/plate_camera/image`. Four short-range sensors on the front, rear, left and right publish on `/ultrasonic/<side>`. Gazebo approximates each ultrasonic cone with a 1.5 m ray fan rather than simulating acoustic echoes. The guard merges their readings with lidar and camera, and stops if a sensor stream goes stale. It checks the swept path of the rover, then publishes the permitted velocity to `/cmd_vel`. The guard slows for obstacles farther ahead, stops for obstacles in the near path, and resumes when the path clears. It stops on stale commands or sensor data. Use `--manual` to test the same guard with keyboard control. The normal patrol follows the curb through the far outer bend at up to 1.25 m/s and slows as steering error increases. OpenAI selects recovery moves on blocked paths.

Camera vision identifies magenta markers placed on the front and rear of the simulated moving cars. It estimates distance from their apparent image width. The lidar handles unmarked obstacles and covers the sides and rear. During automatic patrol, the guard stops when it detects a blocked path. The OpenAI advisor chooses from pull-over arcs validated against current sensor data and the modeled aisle. It can also choose to wait. The guard checks the result again before moving. If there is no valid response or safe choice, the rover stays stopped until the route clears. Pull-over recovery applies to straight aisles.

The `/lot10_encoder_gyro_pose` node estimates the patrol position from four simulated wheel encoder angles on `/wheel_encoders` and gyro Z rate on `/imu/data`. It starts from the known parking position and heading, integrates travel and yaw, and publishes `/patrol/rover_pose`. The patrol stops issuing motion if that pose feed goes stale. `/odom` remains available for diagnostics. It is outside the patrol pose calculation.

Encoder and gyro dead reckoning can drift over a full patrol. A Raspberry Pi deployment needs the measured wheel radius, hardware sensor publishers, and a map reference that can correct accumulated drift. Plate reads still use simulated IDs.


Traffic enters every 15 simulation seconds when the entry is clear and a car slot is available. This gives the rover more opportunities to test obstacle recovery.

## Raspberry Pi 5 integration

Keep the patrol and obstacle guard on the Pi. Use the real lidar driver for `/scan`, encoder/gyro localization for `/patrol/rover_pose`, and a vehicle tracker to replace the simulator's color-marker detector. Set the aisle and rover dimensions from measurements before using the planner with the physical rover. Connect the guard's final `/cmd_vel` output to a Pi-side motor node that forwards wheel targets to the ESP32. The ESP32 should stop its motors if velocity updates time out.

The Pi can call OpenAI for recovery decisions over a network connection. Keep timeouts, live sensor validation, and final velocity control on the Pi.

## Enable OpenAI recovery ranking

The API advisor is opt-in. Enter your own API key in the terminal running the simulation. The key stays in that shell's environment.

```bash
read -rsp 'OpenAI API key: ' OPENAI_API_KEY
export OPENAI_API_KEY
LOT10_LLM=1 bash drive_lot10.sh
```

The default model is `gpt-5-mini`. Set `LOT10_LLM_MODEL` to override it with a compatible Responses API model that supports Structured Outputs and minimal reasoning. The guard requests at most three rankings per blocked-path episode, with a cooldown between attempts. It sends candidate IDs, estimated forward and lateral travel, speeds, turns, and nearest obstacle distances. It sends no camera frames, plate IDs, or exact coordinates.

Look for `LLM recovery ranking enabled` at startup and `executing locally checked LLM choice` when a reply produces a feasible maneuver. The guard pauses for up to 1.5 seconds while waiting for a reply. A timeout, API error, or option that is no longer clear keeps the rover stopped. The advisor can try again with fresh options. A model-selected wait lasts at least one second. Without the API enabled, automatic patrol can still follow a clear path and stops when blocked. Manual mode uses the local obstacle guard without calling the API.
