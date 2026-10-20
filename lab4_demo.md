Demo #1: `ros2 run ece470labs lab4_exec 0.18 -0.1 0.06 -45`

```
Check if the UR3e is in 'Remote' Mode?
    Check if the UR3e is initialized and in 'Normal' state.
    Have you run the ROS2 launch statement?
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.
    
    Press <Enter> to Continue.
[INFO] [1792526936.373829725] [ur3e]: Waiting for initial state updates...

xWgrip: 0.18, yWgrip: -0.1, zWgrip: 0.06, yaw_WgripDegree: -45

Foward kinematics calculated:

[[ 7.07106781e-01  7.07106781e-01 -1.88825667e-17  1.80000000e-01]
 [-7.07106781e-01  7.07106781e-01  2.93662591e-17 -1.00000000e-01]
 [ 1.49175257e-17  1.31092052e-17  1.00000000e+00  6.00000000e-02]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

[INFO] [1792526936.876616868] [ur3e]: Moving to position: [ 122.74110329  -61.35864394  112.19283558 -140.83419165  -90.
   77.74110329]
```

Real Measurements #1: $(17.7, -10.5, 5.5)$ [cm]

Demo #2: `ros2 run ece470labs lab4_exec 0.27 0.33 0.23 90`

```
Check if the UR3e is in 'Remote' Mode?
    Check if the UR3e is initialized and in 'Normal' state.
    Have you run the ROS2 launch statement?
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.
    
    Press <Enter> to Continue.
[INFO] [1792526964.614787221] [ur3e]: Waiting for initial state updates...

xWgrip: 0.27, yWgrip: 0.33, zWgrip: 0.23, yaw_WgripDegree: 90

Foward kinematics calculated:

[[-1.12031793e-16 -1.00000000e+00  2.60821800e-16  2.70000000e-01]
 [ 1.00000000e+00 -1.10006419e-16 -2.79920424e-18  3.30000000e-01]
 [ 2.43348846e-18  1.74871501e-16  1.00000000e+00  2.30000000e-01]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

[INFO] [1792526965.117181637] [ur3e]: Moving to position: [179.38511109 -60.34319423  58.46989844 -88.12670421 -90.
  -0.61488891]
```

Real Measurements #2: $(28.5, 33.0, 23.0)$ [cm]