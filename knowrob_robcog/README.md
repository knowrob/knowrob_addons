# knowrob_robcog #

[Knowrob](http://www.knowrob.org/) package for reasoning on the data gathered using the Unreal Engine virtual environment.

### Usage ###

```
rosrun rosprolog rosprolog knowrob_robcog

roslaunch knowrob_unreal knowrob_robcog_vis.launch
```

### Import Ex ###
```
mongoimport --db unreal_test pathtorawdata
```

### Examples ###

```
u_load_episodes('/media/haidu/hdd2TB/logs/unreal').

u_load_episodes('/home/haidu/sandbox/catkin_ws/src/knowrob_unreal/data_tmp').

owl_parse('/media/haidu/hdd2TB/logs/unreal/u_0_2016.01.27-09.57.24/u_0_EventData.owl').

owl_parse('/home/haidu/sandbox/catkin_ws/src/knowrob_unreal/data_tmp/u_0/u_0_EventData.owl').

owl_parse('/media/haidu/hdd2TB/logs/unreal/oldpf_T_1.owl').
```
