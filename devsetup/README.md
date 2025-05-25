
# Locomotion ROS Noetic Workspace

ROS 1 workspace for the KIT lecture "Humanoid Robots: Locomotion and Whole Body Control". It provides a [devcontainer](https://code.visualstudio.com/docs/devcontainers/containers) config, as well as a repos file for importing all the required packages.

Please read this document carefully before continuing.

## System requirements

A good CPU is strongly recommended, as well as at least 8GB of RAM. If your CPU is not good and you have 8GB of RAM or less, I recommend modifying the build command in below with 
```bash
catkin build -j 1
```
which limits the amount of concurrent used CPU's to 1 (depending on your amount of cores - and not threads - feel free to use 2 or more)

Furthermore, docker needs to be installed, in doubt, refer to https://docs.docker.com/engine/install/ for instructions. You likely need to add your user to the docker group and log out and back in again before docker is available, this can be done in the terminal with `sudo usermod -aG docker <username>`.

This devcontainer is tested in and the instructions provided for Visual Studio Code: https://code.visualstudio.com/

### Start the container using VS Code's devcontainer plugin

Open the workspace in VS Code, make sure you have the devcontainer plugin installed. VS Code might offer to install the Addon itself when opening the folder in VS Code.
Open the command pallet (CTRL+SHIFT+P) and search for `Open in Container`. This may take some time! Be patient and check the logs in the lower left screen corner!

If the devcontainer plugin is unavailable, check your VS Code version, it is for some reason not available in the OSS-version, only in the non-OSS version.


## Pulling the dependencies

Inside the container run

```bash
vcs import < reemc.repos
```

## Using the dev container

In every terminal that you are running things related to ROS in, you need to source two files:
1. `source /opt/ros/noetic/setup.zsh` (the container uses zsh as a shell) will source the general ROS installation.
2. Build the workspace by running `catkin build`. Afterwards, you can
3. source the freshly built workspace by running `source devel/setup.zsh`.

Place ROS packages that you wrote yourself (such as the facepalm package provided with this container configuration) under the /src folder.

If the above commands execute flawlessly, you can, for example, move the facepalm example into the /src folder and rerun `catkin build`.

## Getting the locomotion packages from Ilias

In addition to the PAL (robot company) ROS packages you will also need some custom packages!
Download sheet1_reference_python from Exercise1/Ilias and copy the package loco_exercises into the /src as well.

## Building the workspace

This might take some time: Build your workspace with
```bash
catkin build
```
Be sure to run this in the root (so above src which should contain all your ROS packages)


## Verify everything works


Open three terminals (use vscode or tmux or install terminator using `sudo apt install terminator`).
Make sure your workspace is sourced. In on terminal, start the simulation using 
```bash
roslaunch reemc_gazebo reemc_empty_world.launch
```

and in the other the controllers with
```bash
roslaunch reemc_controller_configuration joint_trajectory_controllers.launch
```

In the third terminal, run the example code with
```bash
rosrun facepalm facepalm.py
```

## Troubleshooting

- no gui
    - did you run `xhost +` on your host system?
- still no gui
    - first, install `x11-apps` and `mesa-utils` in the container
    ```bash
    sudo apt update
    sudo apt install x11-apps mesa-utils
    ```
    - run `xeyes` to verify that x-socket forwarding is working. You should see a window open with
      two eyes following your cursor
        - if it says something like `cant open display :1`, check the `/tmp/.X11-unix` folder using
          `ls /tmp/.X11-unix` and try out every display socket listed there. For example `ls` would
          show something like `X0 X1 X2 X3`. Test each socket by running `DISPLAY=:N xeyes`, where
          `N` is the number of a socket, so for X1 you would write something like `DISPLAY=:1
          xeyes`.
          If you found a working socket, from now one write `export DISPLAY=:N` in every new shell
          session in the container.
    - run `glxgears` to verify that hardware acceleration is working. You should see a window open
      with a bunch of gears turning. If you don't, or only see a black window, your hardware
      acceleration is broken and stuff like rviz or gazebo won't work
        - if you have a Nvidia GPU and some sort of non-default driver setup (e.g. everything that
          has to do with cuda), make sure to install and setup the [`nvidia-container-toolkit`](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- build crashes
    - if you are using docker desktop, try increasing the RAM assigned to your docker
    - build with fewer cores using the `-j` flag for catkin, e.g. `catkin build -j 4`

- something with ros didn't work
    - did you source the workspace?
