## legged_wbc improved

We find that in `legged_wbc`, the optimization variable is $[\dot{u}, F, \tau]^T$, whose dimension is 42. High-dimensional optimization variables increase the computing time of `legged_wbc`. In addition, we also found that only the desired acceleration was used in `legged_wbc`, without velocity and position errors, which may lead to imprecise performance of the robot. Therefore, we rewrote `legged_wbc` according to xx, and the improvements are as follows:

1. Reduce the dimension of `legged_wbc` optimization variables. The original optimization variable is $[\dot{u}, F, \tau]^T$, and our improved optimization variable is $[\dot{u}, F]^T$. The dimension is reduced by 12 compared to before.
2. Redesign the control law.  We added the position and speed errors to the control law. In addition, we designed height, attitude and xy position as three subtasks instead of synthesizing one big task.

The performance comparison is as follows: We test on NUC11PAHi7 with the robot running in gazebo. The maximum number of iterations for the QP solver is set to 50.

The original result was

```
HierarchicalWbc
Maximum : 4.57273[ms].
Average : 0.531365[ms].
gazebo realtime factor 0.86

WeightedWbc
Maximum : 4.49292[ms].
Average : 0.330465[ms].
gazebo realtime factor 0.93
```

Now the result is

```
HierarchicalWbc
Maximum : 2.3478[ms].
Average : 0.392037[ms].
gazebo realtime factor 0.93

WeightedWbc
Maximum : 5.57294[ms].
Average : 0.216672[ms].
gazebo realtime factor 0.95
```

