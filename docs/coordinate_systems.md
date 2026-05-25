# Coordinate Systems and Frame Conventions

## Local ENU Frame

References, controller tracking, metrics, and exported local positions use the local ENU frame:

- X: East
- Y: North
- Z: Up
- Origin: launch pad

RocketPy integrates absolute positions including launch elevation. The simulator converts to local ENU by subtracting the launch position:

```text
position_enu_m = position_asl_m - launch_position_asl_m
```

## RocketPy State Vector

RocketPy controller callbacks receive:

```text
[x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
```

The quaternion is stored and exported in scalar-first order:

```text
[w, x, y, z]
```

## Body Frame

The controller uses the RocketPy body convention:

- body X: pitch axis
- body Y: yaw axis
- body Z: longitudinal roll axis, tail to nose

The desired attitude quaternion maps ENU to body and aligns body +Z with the commanded nose direction.

## Attitude Error

The controller computes:

```text
q_error = q_ref * conjugate(q_real)
```

The error components used for control are:

- pitch: `q_error[1]`
- yaw: `q_error[2]`
- roll: `q_error[3]`

## Fin Numbering And Mixer

The four fin commands are stored as:

```text
[delta0, delta1, delta2, delta3]
```

The mixer follows the Siouris cruciform-fin convention. In code the array is
zero-indexed, so `delta0..delta3` correspond to physical fins `d1..d4`:

```text
delta0 =  pitch + roll
delta1 =  yaw   + roll
delta2 = -pitch + roll
delta3 = -yaw   + roll
```

`FinAdapter` extracts equivalent control components as:

```text
delta_pitch = (delta0 - delta2) / 2
delta_yaw   = (delta1 - delta3) / 2
delta_roll  = mean(delta0, delta1, delta2, delta3)
```

## Summary Table

| Quantity | Frame | Units |
| --- | --- | --- |
| Position | local ENU | m |
| Velocity | local ENU | m/s |
| Reference | local ENU | m, m/s |
| Attitude quaternion | ENU to body | unit quaternion |
| Body rates | body | rad/s |
| Fin deflection | fin/body convention | rad |
