# f1tenth_race_metrics

To run the code, start the node and gym/particle filter. The code will wait for a the `/initial_pose` command before it beings recording. It also wont begin recording until the car starts moving. Once you finish, you can stop the node by killing it with CTRL+C. The code will also stop recording automatically if the velocity is below `stall_speed_threshold_m_s` for `stall_duration_s` seconds in the event of a crash. 

## Sim:

```bash
ros2 launch f1tenth_race_metrics metrics.launch.py
```

## Hardware:

```bash
ros2 launch f1tenth_race_metrics metrics.launch.py use_hardware:=true
```