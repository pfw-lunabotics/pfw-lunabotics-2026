# PFW Lunabotics — Mission Control GUI (operator VM)

A PyQt5 desktop GUI that runs on the operator VM (`192.168.0.100`) and drives
the Jetson over the same DDS network + SSH that you already have working.

## What it gives you

A single window with four resizable panes plus a global E-STOP button:

| Pane | What it does |
|------|--------------|
| **1 · Autonomous** | Starts/stops the full hardware bringup (the same launch that `scripts/scenario2_full_mission.sh` runs on the Jetson). Setpose + setlayout inputs, then **START MISSION** calls `/mission/start`. Live stdout/stderr from the bringup streams into the log. |
| **2 · Manual Teleop** | Mimics the `teleop_keyboard` OBSERVER ↔ MANUAL toggle. Click the toggle (or it's the equivalent of pressing `m`). In MANUAL, **W/A/S/D** publishes `geometry_msgs/Twist` to `/cmd_vel` at 20 Hz with adjustable max linear/angular. Excavation buttons (Dig / Dump / Stow), continuous Actuator UP / DOWN, Belt OFF. |
| **3 · Robot Health** | Live subscribers to `/pico/status`, `/system/health`, `/mission/state`, `/mission/status`, `/perception/current_zone`, `/perception/localization_quality`, `/control/status`, and `/estop`. Also pings the Jetson and shows live `ros2 node list` count. |
| **4 · Debug Terminal** | Type any command — runs over SSH with ROS2 already sourced. Quick buttons for `ros2 topic list`, `node list`, `topic echo`, `topic hz`, "clear estop". Up/Down arrows = command history. |

The big red **E-STOP** button at the top is always visible. It publishes
`std_msgs/Bool true` to **both** `/estop` and `/safety/estop`, freezes manual
teleop, and reflects the latched state of `/estop` if anything else (teleop on
the Jetson, mission controller) flips it. Pressing **Escape** toggles it too.

## How it talks to the Jetson

```
+-----------+   ROS2 / DDS (UDP multicast over router)   +----------+
|           |  /estop, /cmd_vel, /mission/*, /pico/* ... |          |
|    VM     |<------------------------------------------>| Jetson   |
| (PyQt5)   |                                            | (bringup |
|           |   SSH (passwordless, key auth)             |  + nodes)|
|           |---------------------------------------->----|          |
+-----------+   start/kill bringup, ad-hoc commands      +----------+
```

- **All real-time ROS2 IO** (publishing cmd_vel, e-stop, service calls,
  subscribing to topics) uses **rclpy directly from the VM**. Because both
  machines run `ROS_DOMAIN_ID=0` on the same subnet, the topics are visible
  natively — exactly as you confirmed when you tested `ros2 topic pub` from
  the VM.
- **Long-running shell stuff** (the bringup itself, the ad-hoc terminal) goes
  over SSH. The GUI launches `bash -lc '...'` so ROS2 is sourced on each
  invocation.
- Only standard ROS2 message types are used (`std_msgs`, `geometry_msgs`,
  `std_srvs`), so **the VM does not need the `luna_msgs` package or the
  workspace built** — only `ros-humble-ros-base`.

## Pico bridge + servo driver stay running

Both are launched by `bringup.launch.py` (in the autonomy pane). The GUI never
kills them when you toggle Manual mode — manual teleop just starts publishing
`/cmd_vel`, which the running `cmd_vel_relay` forwards to `/cmd_vel_safe` and
then to the still-running `pico_bridge`. Switching back to OBSERVER simply
stops publishing; the autonomous mission (if running) continues.

## Setup on the VM (split into 3 scripts so only the first needs internet)

```bash
cd mission_control_vm
chmod +x setup_vm_deps.sh setup_ethernet.sh setup_ssh_keys.sh run_gui.sh
```

| Step | Script | Internet? | Where |
|------|--------|-----------|-------|
| 1 | `./setup_vm_deps.sh`   | **YES** (one time, at home) | VM |
| 2 | `sudo ./setup_ethernet.sh` | no | **BOTH** Jetson and VM |
| 3 | `./setup_ssh_keys.sh`  | no | VM |
| 4 | `./run_gui.sh`         | no | VM |

- **`setup_vm_deps.sh`** — `apt install python3-pyqt5 openssh-client iputils-ping`. This is the **only** script that needs internet. Run once at home and never again.
- **`setup_ethernet.sh`** — auto-detects which side it's on by looking for `enP8p1s0` (the Jetson's Tegra-named iface). Sets Jetson to **192.168.0.200/24** and VM to **192.168.0.100/24**. Uses NetworkManager when available (persistent across reboot), falls back to plain `ip addr` otherwise. Run on the Jetson too: `scp setup_ethernet.sh lunabotics@192.168.0.200:~ && ssh lunabotics@192.168.0.200 'sudo ./setup_ethernet.sh'`.
- **`setup_ssh_keys.sh`** — `ssh-keygen` if needed + `ssh-copy-id`. One password prompt, then never again.

The legacy `setup_vm.sh` (does steps 1+3 in one go) is left in place for backwards compatibility but the split scripts are preferred — you only have to run the internet-dependent one once.

## Running the GUI

```bash
./run_gui.sh
```

You can also override the host live in the top bar — it's a text field, no
restart needed.

## Typical session

1. Launch the GUI on the VM.
2. **Top bar** — confirm Jetson user/host. Check that pane 3 shows "reachable"
   and a non-zero node count.
3. **Pane 1** — click **Start Bringup**. Watch the log fill up. After ~15 s
   you should see Nav2/perception/Point-LIO come up; pane 3 should start
   showing live values for `/pico/status`, `/mission/state`, etc.
4. **Pane 1** — set X / Y / Yaw (matches the `setpose` command from
   scenario2), pick arena layout, click **setpose** and **setlayout**.
5. **Pane 1** — click **START MISSION**. That's just `/mission/start` over the
   network.
6. While it runs, watch pane 3. If anything looks wrong, hit **E-STOP**.
7. To take over: click the **MANUAL** toggle in pane 2, then drive with WASD.
   Click MANUAL again to release back to the autonomous system.
8. To kill: **Stop Mission** (graceful) and/or **Kill Bringup** (full
   teardown of all nodes).

## Files

| File | Purpose |
|------|---------|
| `mission_control.py` | The whole GUI in one file (~700 lines of PyQt5 + rclpy). |
| `setup_vm.sh` | One-shot installer for VM deps + SSH key. |
| `run_gui.sh` | Generated by `setup_vm.sh`; sources ROS2 and launches the GUI. |
| `README.md` | This file. |

## Troubleshooting

**The GUI launches but pane 3 shows no values**
- The VM hasn't joined the same DDS network as the Jetson, or the bringup
  isn't running yet. From a terminal on the VM:
  ```
  source /opt/ros/humble/setup.bash
  ros2 topic list | grep mission
  ros2 topic echo --once /pico/status
  ```
  If `ros2 topic list` is empty, check `ROS_DOMAIN_ID` is `0` on both sides
  and that nothing is blocking UDP discovery on the router.

**"Start Bringup" fails immediately**
- Open pane 4 (Debug Terminal) and run `whoami` and `ls ~/pfw-lunabotics`.
  If either fails, the SSH path or workspace path is wrong — fix the host
  in the top bar or edit `WORKSPACE` at the top of `mission_control.py`.

**E-STOP doesn't stop the wheels**
- Confirm `/safety/estop` is honored by the running `cmd_vel_relay` — in
  pane 4 run `ros2 topic echo --once /control/status`. It should switch from
  `RUNNING ...` to `ESTOP` within a few ms of the E-STOP press.

**Manual teleop publishes but wheels don't move**
- See the "Motion-Path Debug Guide" memory; in pane 4 the "Quick buttons" have
  `hz lidar` and `mission` plus the bringup status — also run
  `ros2 topic hz /cmd_vel_safe` and `ros2 topic echo --once /pico/status`.

**"Kill Bringup" leaves stale nodes**
- In pane 4: `pkill -9 -f ros2`. The GUI uses SIGINT + SIGKILL via the PID
  file in `/tmp/luna_bringup.pid` plus a `pkill -f bringup.launch.py`, but a
  hard pkill from the terminal is the cleanup of last resort.

## Why not embed the existing `teleop_keyboard.py`?

`teleop_keyboard.py` is a curses-style raw-tty Python program. Embedding it in
a Qt widget would require a full PTY terminal emulator (e.g. `qtermwidget`)
and would still steal keys from the rest of the GUI. The GUI replicates the
*exact* OBSERVER ↔ MANUAL behaviour you use from the keyboard (W/A/S/D
drive, MANUAL button = pressing `m`, Space stops, plus buttons for the
excavation / actuator / dump that the existing teleop also has). Pico bridge
and servo driver are unaffected — they stay up via the bringup.

If you do want the original teleop running over SSH for a session, pane 4
will take it:
```
ros2 run lunabotics_control teleop_keyboard
```
…but pane 4 isn't a full terminal, so the raw-tty key capture won't work
there. For that, just `ssh -t lunabotics@192.168.0.200 'ros2 run
lunabotics_control teleop_keyboard'` in a separate terminal on the VM.
