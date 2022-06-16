# ros2_test_subscriber

create and destruct subscriber at runtime.

### normal case

```bash
ros2 launch ros2_test_subscriber test_subscriber.launch.py use_dedicated_thread:=False
```

actual output 
```
[test_subscriber-1] [INFO] [1655367815.984070995] [task_server]: activate
[test_subscriber-1] [INFO] [1655367815.984688260] [task_client]: client success
[test_subscriber-1] [INFO] [1655367816.478977047] [task_server]: recv data: 2.
[test_subscriber-1] [INFO] [1655367816.978938729] [task_server]: recv data: 3.
[test_subscriber-1] [INFO] [1655367817.478962625] [task_server]: recv data: 4.
[test_subscriber-1] [INFO] [1655367817.978948932] [task_server]: recv data: 5.
[test_subscriber-1] [INFO] [1655367818.478906542] [task_server]: recv data: 6.
[test_subscriber-1] [INFO] [1655367818.978931629] [task_server]: recv data: 7.
[test_subscriber-1] [INFO] [1655367818.984846362] [task_server]: deactivate
[test_subscriber-1] [INFO] [1655367818.985228253] [task_client]: client success
[test_subscriber-1] [INFO] [1655367821.985405036] [task_server]: activate
[test_subscriber-1] [INFO] [1655367821.985847860] [task_client]: client success
[test_subscriber-1] [INFO] [1655367822.478968309] [task_server]: recv data: 14.
[test_subscriber-1] [INFO] [1655367822.978955636] [task_server]: recv data: 15.
[test_subscriber-1] [INFO] [1655367823.478954584] [task_server]: recv data: 16.
[test_subscriber-1] [INFO] [1655367823.978956295] [task_server]: recv data: 17.
...
```
### use dedicated thread for subscriber

```bash
ros2 launch ros2_test_subscriber test_subscriber.launch.py use_dedicated_thread:=True
```

actual output (ubuntu22.04 ROS Rolling)
```
[INFO] [test_subscriber-1]: process started with pid [1366]
[test_subscriber-1] [INFO] [1655367910.706412298] [task_server]: activate
[test_subscriber-1] [INFO] [1655367910.706964925] [task_client]: client success
[test_subscriber-1] [INFO] [1655367913.707151741] [task_server]: deactivate
[test_subscriber-1] [INFO] [1655367913.707534238] [task_client]: client success
[test_subscriber-1] [INFO] [1655367916.707709411] [task_server]: activate
[test_subscriber-1] [INFO] [1655367916.708197168] [task_client]: client success
[test_subscriber-1] [INFO] [1655367919.708380884] [task_server]: deactivate
[test_subscriber-1] [INFO] [1655367919.708659300] [task_client]: client success
[test_subscriber-1] [INFO] [1655367922.708833489] [task_server]: activate
[test_subscriber-1] [INFO] [1655367922.709321480] [task_client]: client success
```
