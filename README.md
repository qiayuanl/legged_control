## legged_wbc improved

Hi, , thanks for your open source. This submission is mainly aimed at wbc.

We find that in `legged_wbc`, the optimization variable is $[\dot{u}, F, \tau]^T$, whose dimension is 42. High-dimensional optimization variables increase the computing time of `legged_wbc`. In addition, we also found that only the desired acceleration was used in `legged_wbc`, without velocity and position errors, which may lead to imprecise performance of the robot. Therefore, we rewrote `legged_wbc` according to cited papers, and the improvements are as follows:

1. Reduce the dimension of `legged_wbc` optimization variables. The original optimization variable is $[\dot{u}, F, \tau]^T$, and our improved optimization variable is $[\dot{u}, F]^T$. The dimension is reduced by 12 compared to before.
2. Redesign the control law.  We added the position and speed errors to the control law. In addition, we designed height, attitude and xy position as three subtasks instead of synthesizing one big task.

The performance comparison is as follows: We tested on NUC11PAHi7 with the robot running in gazebo. The maximum number of iterations for the QP solver was set to 50.

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

Now the result was

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



[1] Bellicoso, C. D., Gehring, C., Hwangbo, J., Fankhauser, P., & Hutter, M. (2016, November). Perception-less terrain adaptation through whole body control and hierarchical optimization. In *2016 IEEE-RAS 16th International Conference on Humanoid Robots (Humanoids)* (pp. 558-564). IEEE.

[2] Bellicoso, C. D., Jenelten, F., Fankhauser, P., Gehring, C., Hwangbo, J., & Hutter, M. (2017, September). Dynamic locomotion and whole-body control for quadrupedal robots. In *2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)* (pp. 3359-3365). IEEE.
