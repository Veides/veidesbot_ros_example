# VeidesBot ROS example

This repository contains basic packages to make start working with Veides easier on example of a simple robot and shows how [Veides Agent ROS node](https://github.com/Veides/veides_agent_ros.git) might be used to develop a device with [ROS](http://wiki.ros.org).

## Installation

We assume you have [ROS Noetic](http://wiki.ros.org/noetic/Installation) already installed.

### Create workspace

First, you need to create a catkin workspace to be able to run the simulation. You can skip this point if you already have workspace prepared.

```bash
cd ~
mkdir -p catkin_workspace/src
cd catkin_workspace/src
catkin_init_workspace
cd ..
catkin_make
```

### Setup environment

```bash
source ~/catkin_workspace/devel/setup.sh
```

Please remember that above command needs to be executed each time you open new terminal window.

### Setup VeidesBot example

Clone this repository and it's submodules to your workspace:

```bash
cd ~/catkin_workspace/src
git clone --recursive https://github.com/Veides/veidesbot_ros_example.git
```

Install dependencies:

```bash
cd ~/catkin_workspace
rosdep install --from-paths src --ignore-src -r -y
pip3 install veides-agent-sdk
```

Build and install the workspace:

```bash
cd ~/catkin_workspace
catkin_make
catkin_make install
```

## Usage

Package `veidesbot_gazebo` contains 2 launch files:

1. full.launch
2. full_nogui.launch

Both starts all necessary nodes to VeidesBot operational. What's the difference then? The first one runs Gazebo and RViz so you can see that is happening while the other do not run RViz at all and starts Gazebo in non-gui mode meaning the simulation is working but without visualization.  

If you want to start Gazebo only for visualization, just comment out RViz node from `full.launch` file.

### Configuration

In package `veidesbot_platform` you can find file `veidesbot.launch`. By default, this launch file use environment variables to load values for parameters passed to `veides_agent_ros` node. You can either modify launch file and put values directly or use environment file.

If you'd like to use environment file, prepare one so it looks as follows:

```bash
$ cat .env
export VEIDES_AGENT_NAME=veidesbot

export VEIDES_CLIENT_HOST=host to connect to
export VEIDES_AGENT_CLIENT_ID=agent client id
export VEIDES_AGENT_KEY=agent key
export VEIDES_AGENT_SECRET_KEY=agent secret key
```

Agent and connection properties can be obtained from the Console. When `.env` file is ready, type:

```bash
source .env
```

### Run the simulation

When parameters are configured you can launch the simulation:

```bash
roslaunch veidesbot_gazebo full.launch
```

or viewless version:

```bash
roslaunch veidesbot_gazebo full_nogui.launch
```

Please note that Veides interaction parts may require agent to be configured for particular case to work.
