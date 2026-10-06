Demo #1: `ros2 run ece470labs lab3_exec 6.37 -69.90 125.67 -56.76 -88.54 89.90`

Output #1:
```
Check if the UR3e is in 'Remote' Mode?
    Check if the UR3e is initialized and in 'Normal' state.
    Have you run the ROS2 launch statement?
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.
    
    Press <Enter> to Continue.
[INFO] [1791317893.488363994] [ur3e]: Waiting for initial state updates...

theta1: 6.37, theta2: -69.90, theta3: 125.67, theta4: -56.76, theta5: -88.54, theta6: 89.90

Foward kinematics calculated:

[[ 0.99348194 -0.11308419 -0.01433879  0.17765632]
 [ 0.11266576  0.99325954 -0.02723808  0.3223578 ]
 [ 0.01732234  0.02544505  0.99952613  0.06650047]
 [ 0.          0.          0.          1.        ]]

[INFO] [1791317894.011710588] [ur3e]: Moving to position: [ 186.37  -69.9   125.67 -146.76  -88.54   89.9 ]
```

Demo #2: `ros2 run ece470labs lab3_exec -46.73 -75.65 98.23 -59.53 -112.40 89.88`

Output #2:
```
Check if the UR3e is in 'Remote' Mode?
    Check if the UR3e is initialized and in 'Normal' state.
    Have you run the ROS2 launch statement?
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.
    
    Press <Enter> to Continue.
[WARN] [1791317982.718204471] [ur3e]: IO service not available, waiting...
[INFO] [1791317983.469876889] [ur3e]: Waiting for initial state updates...

theta1: -46.73, theta2: -75.65, theta3: 98.23, theta4: -59.53, theta5: -112.40, theta6: 89.88

Foward kinematics calculated:

[[ 0.54951183  0.82905412 -0.10346991  0.21322662]
 [-0.58091485  0.4681441   0.66586713 -0.12853007]
 [ 0.60047872 -0.30579466  0.73886057  0.28825358]
 [ 0.          0.          0.          1.        ]]

[INFO] [1791317983.972426387] [ur3e]: Moving to position: [ 133.27  -75.65   98.23 -149.53 -112.4    89.88]
```