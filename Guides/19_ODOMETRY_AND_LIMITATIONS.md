# 10. Odometry and Limitations

## 10.1 Inputs

Current position estimation uses:

- Hall wheel pulses for distance;
- MPU6050 gyro Z for yaw integration.

The base calibration values are:

```text
wheel circumference = 20.42 cm
8 ticks/revolution
```

These are inherited from the rover base and should be experimentally calibrated for the physical rover.

## 10.2 Position model

The rover integrates incremental distance along its current heading to estimate:

```text
x
 y
heading
```

The dashboard draws this as a relative path/trail.

## 10.3 Odometer confidence

The DevKit reports an odometry-confidence value that accounts for MPU readiness and Hall-feedback freshness during motion.

Treat it as a diagnostic confidence indicator, not a calibrated statistical probability.

## 10.4 Important limitations

With only one Hall channel:

- left/right wheel displacement is not independently observed;
- differential-turn geometry cannot be fully recovered from wheel pulses alone;
- wheel slip can distort distance;
- gyro bias creates heading drift;
- long-duration position error accumulates.

The map is therefore a relative visualization, not a GPS-grade position estimate.

## 10.5 Improving accuracy without new hardware

You can still improve the estimate by:

- measuring actual wheel circumference under load;
- measuring real ticks per revolution;
- calibrating gyro bias carefully;
- using stationary periods for conservative bias adaptation;
- comparing commanded turn behavior with gyro turn rate;
- detecting large wheel-vs-gyro inconsistencies and reducing odometry confidence.

## 10.6 Safe interpretation

Do not use the dead-reckoned map as a guarantee of physical location after a long run. Use it for relative path inspection, maneuver debugging and short-duration navigation visualization.
