
# FAQ

If you have solutions for common pitfalls (maybe things you experienced yourself) please add those to this file.

## Sourcing

After you compiled this repository, respectively ROS workspace with `catkin_make` and want to run something from the terminal, 
you always have to `source` the ROS workspace beforehand in **each terminal** or you put a corresponding command into your `.bash_rc`.

Sourcing is done from within this workspace with:

`source devel/setup.bash`

This is not necessary on the real car because it is already sourced in the .bash_rc, see below.

## Connecting to a remote ROS master/core

ROS allows to connect nodes across multiple machines on the network. To achieve this it is necessary to share one common ROS
master, respectively core. We have configured a setup that makes use of zeroconf network identification to avoid messing around
with changing IPs.

In our setup the ROS master/core is running on the cars. The necessary setup is done by sourcing the script `scripts/remoteHostNetwork.sh`,
which is already done in the `.bashrc` on the cars.

If you want to connect from any remote machine (you can run all nodes remotely except for the ones directly accessing the hardware devices (sensors/actuators)),
you have to source the script `scripts/remoteClientNetwork.sh` with a parameter identifying the host name of the machine running the ROS master.

Example to connect to our 4WD platform:

```
source scripts/remoteClientNetwork.sh jetson2
```

Example to connect to our 6WD platform.

```
source scripts/remoteClientNetwork.sh jetson3
```

The sourcing command has to be executed on every terminal that is connecting to the remote ROS master additionally to the 
sourcing of the workspace itself.


### Example .bashrc Extensions

```
# general ROS setup sourcing (part of official install tutorial)
source /opt/ros/melodic/setup.bash

# Some additional environment variables to find libraries
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/melodic/lib/x86_64-linux-gnu
export PYTHONPATH=$PYTHONPATH:/usr/local/lib

# Sourcing for the remote ROS master setup
source /home/jetson2/software_integration_new/scripts/remoteHostNetwork.sh

# Sourcing of the workspace.
source /home/jetson2/software_integration_new/devel/setup.bash
```

## Git

### Pulling Upstream improvements

If you want to incorporate commits that have been made available on the main repository of the course, please follow these steps:

`git remote add upstream [GITUPSTREAMREPOURL]`
replace `[GITUPSTREAMREPOURL]` with the main repository of your class.

`git fetch`

To merge the latest version from master:
`git pull upstream master`

### Submodule certificate problems

If you are receiving certificate errors 'server certificate verification failed' while updating the submodules you can workaround this issue by disabling the SSL certificate verification

`git config --global http.sslVerify false`

If you remove the option `--global` it will only affect the current git repository

### Caching credentials 

If you want to avoid to type your git credentials for all submodules repeatedly you can configure git to cache this

`git config --global credential.helper 'cache --timeout=600'`

`600` specifies the time in seconds to cache previously typed credentials.

## Useful ROS commands

* `rosrun [PACKAGENAME] [NODENAME]`: Launches a ROS node.
* `roslauch [PACKAGENAME] [LAUNCHFILENAME]`: Executes all ROS nodes/scripts/config contained in a launch file. Allows to maintain more complex runtime setups.
* `rostopic`: List all available topics (param `list`), publish (param `pub`) and subscribe (param `echo`)  to topics
* `rosservice`: List all available services (param `list`), call a service (param `call`), etc.
* `roscd [PACKAGENAME]`: Takes as an argument the package name and changes to its directory.
* `rqt`: Modular UI that can be customized with various plugins from the ROS installation to monitor, inspect, manipulate the ROS runtime system. E.g. Monitoring topic content, visualising nodes and their connection, reading logs and bags, ...
* `rviz`: Another modular UI that enables to visualize and interact with sensors and actuators in 2D/3D.

## Questions and Answers

* Do I have to run `catkin_make` every time before I run something from the project?
    * You only need to run `catkin_make` if you have edited C/C++ code, CMakeList.txt or any service (*.srv), message (*.msg) or action file. If you are editing launch files or python code, you can directly run everything.
* If I have problems compiling a certain package, which I might even not need (e.g. sensor package on a remote machine), how can I disable it to successfully compile the workspace?
    * To disable the compilation of a package just create a file with the name `CATKIN_IGNORE` within the package directory.