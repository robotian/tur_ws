# TUR_WS 

This project is for multiple tethered ROV simulation using ROS2 and Gazebo. This repo is forked from [ORCA4](https://github.com/clydemcqueen/orca4).


## Development Environment

Setting VSCode for ROS development using a Dev Container is explained well in this [repo](https://github.com/athackst/vscode_ros2_workspace).

Recommended Tools:
* [Visual Studio Code](https://code.visualstudio.com/docs/setup/linux)
* [Docker](https://docs.docker.com/engine/install/ubuntu/)
* Git(of course) 
* [NVIDIA Contianer toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)(Optional)

### Account setup for Docker
~~~
sudo groupadd docker
sudo usermod -aG docker $USER
~~~

Log out and log in back 
~~~
newgrp docker
~~~

Test the Docker installation
~~~
docker run hello-world
~~~

The output should be similar to the below.
~~~
Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
    (amd64)
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.

To try something more ambitious, you can run an Ubuntu container with:
 $ docker run -it ubuntu bash

Share images, automate workflows, and more with a free Docker ID:
 https://hub.docker.com/

For more examples and ideas, visit:
 https://docs.docker.com/get-started/

~~~

### Nvidia toolkit setup
If you have Nvidia GPUs and want to use them in a docker container, you will need to install the Nvidia container toolkit. Follow the instruction provided [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). 

After you install and configure the toolkit and install an NVIDIA GPU Driver, you can verify your installation by running a sample workload.
~~~
sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
~~~
Your output should resemble the following output:
~~~
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.86.10    Driver Version: 535.86.10    CUDA Version: 12.2     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  Tesla T4            On   | 00000000:00:1E.0 Off |                    0 |
| N/A   34C    P8     9W /  70W |      0MiB / 15109MiB |      0%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+

+-----------------------------------------------------------------------------+
| Processes:                                                                  |
|  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
|        ID   ID                                                   Usage      |
|=============================================================================|
|  No running processes found                                                 |
+-----------------------------------------------------------------------------+
~~~



## Installation

See the [Dockerfile](.devcontainer/Dockerfile) for installation details.

Installed packages in the docker file:
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Gazebo Garden 7.1.0](https://gazebosim.org/docs/garden/install)
* [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
* [ArduSub](https://ardupilot.org/dev/docs/building-setup-linux.html)


Create a workspace folder and clone this repo with the submodules
~~~
mkdir workspace
cd workspace
git clone -b humble git@github.com:robotian/tur_ws.git --recurse-submodules
~~~

Open the project folder 'tur_ws' with VSCode and 'Reopen in Container.' It will start the Docker image building process and take a while. 
If the building process is successful, you should be able to open a terminal in the container. 

Install MAVROS dependency
~~~
sudo src/mavros/mavros/scripts/install_geographiclib_datasets.sh
~~~

Install ROS package dependencies by running a bash script
~~~
./ws_setup.sh
~~~


Build the 'tur_ws' package
~~~
./ws_build.sh
~~~


Load the environment variables
~~~
source src/multiple_orca/setup_container.bash
~~~















# VSCode ROS2 Workspace Template

This template will get you set up using ROS2 with VSCode as your IDE.

See [how I develop with vscode and ros2](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for a more in-depth look on how to use this workspace.

## Features

### Style

ROS2-approved formatters are included in the IDE.  

* **c++** uncrustify; config from `ament_uncrustify`
* **python** autopep8; vscode settings consistent with the [style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/)

### Tasks

There are many pre-defined tasks, see [`.vscode/tasks.json`](.vscode/tasks.json) for a complete listing.  Feel free to adjust them to suit your needs.  

Take a look at [how I develop using tasks](https://www.allisonthackston.com/articles/vscode_tasks.html) for an idea on how I use tasks in my development.

### Debugging

This template sets up debugging for python files, gdb for cpp programs and ROS launch files.  See [`.vscode/launch.json`](.vscode/launch.json) for configuration details.

### Continuous Integration

The template also comes with basic continuous integration set up. See [`.github/workflows/ros.yaml`](/.github/workflows/ros.yaml).

To remove a linter just delete it's name from this line:

```yaml
      matrix:
          linter: [cppcheck, cpplint, uncrustify, lint_cmake, xmllint, flake8, pep257]
```

## How to use this template

### Prerequisites

You should already have Docker and VSCode with the remote containers plugin installed on your system.

* [docker](https://docs.docker.com/engine/install/)
* [vscode](https://code.visualstudio.com/)
* [vscode remote containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Get the template

Click on "use this template"

![template_use](https://user-images.githubusercontent.com/6098197/91331899-43f23b80-e780-11ea-92c8-b4665ce126f1.png)

### Create your repository

On the next dialog, name the repository you would like to start and decide if you want all of the branches, or just the latest LTS: humble.

![template_new](https://user-images.githubusercontent.com/6098197/91332035-713ee980-e780-11ea-81d3-13b170f568b0.png)

Github will then create a new repository with the contents of this one in your account.  It grabs the latest changes as "initial commit".

### Clone your repo

Now you can clone your repo as normal

![template_download](https://user-images.githubusercontent.com/6098197/91332342-e4e0f680-e780-11ea-9525-49b0afa0e4bb.png)

### Open it in vscode

Now that you've cloned your repo onto your computer, you can open it in VSCode (File->Open Folder). 

When you open it for the first time, you should see a little popup that asks you if you would like to open it in a container.  Say yes!

![template_vscode](https://user-images.githubusercontent.com/6098197/91332551-36898100-e781-11ea-9080-729964373719.png)

If you don't see the pop-up, click on the little green square in the bottom left corner, which should bring up the container dialog

![template_vscode_bottom](https://user-images.githubusercontent.com/6098197/91332638-5d47b780-e781-11ea-9fb6-4d134dbfc464.png)

In the dialog, select "Remote Containers: Reopen in container"

VSCode will build the dockerfile inside of `.devcontainer` for you.  If you open a terminal inside VSCode (Terminal->New Terminal), you should see that your username has been changed to `ros`, and the bottom left green corner should say "Dev Container"

![template_container](https://user-images.githubusercontent.com/6098197/91332895-adbf1500-e781-11ea-8afc-7a22a5340d4a.png)

### Update the template with your code

1. Specify the repositories you want to include in your workspace in `src/ros2.repos` or delete `src/ros2.repos` and develop directly within the workspace.
2. If you are using a `ros2.repos` file, import the contents `Terminal->Run Task..->import from workspace file`
3. Install dependencies `Terminal->Run Task..->install dependencies`
4. (optional) Adjust scripts to your liking.  These scripts are used both within tasks and CI.
   * `setup.sh` The setup commands for your code.  Default to import workspace and install dependencies.
   * `build.sh` The build commands for your code.  Default to `--merge-install` and `--symlink-install`
   * `test.sh` The test commands for your code.
5. Develop!


## FAQ

### WSL2

#### The gui doesn't show up

This is likely because the DISPLAY environment variable is not getting set properly.

1. Find out what your DISPLAY variable should be

      In your WSL2 Ubuntu instance

      ```
      echo $DISPLAY
      ```

2. Copy that value into the `.devcontainer/devcontainer.json` file

      ```jsonc
      	"containerEnv": {
		      "DISPLAY": ":0",
         }
      ```

#### I want to use vGPU

If you want to access the vGPU through WSL2, you'll need to add additional components to the `.devcontainer/devcontainer.json` file in accordance to [these directions](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md)

```jsonc
	"runArgs": [
		"--network=host",
		"--cap-add=SYS_PTRACE",
		"--security-opt=seccomp:unconfined",
		"--security-opt=apparmor:unconfined",
		"--volume=/tmp/.X11-unix:/tmp/.X11-unix",
		"--volume=/mnt/wslg:/mnt/wslg",
		"--volume=/usr/lib/wsl:/usr/lib/wsl",
		"--device=/dev/dxg",
      		"--gpus=all"
	],
	"containerEnv": {
		"DISPLAY": "${localEnv:DISPLAY}", // Needed for GUI try ":0" for windows
		"WAYLAND_DISPLAY": "${localEnv:WAYLAND_DISPLAY}",
		"XDG_RUNTIME_DIR": "${localEnv:XDG_RUNTIME_DIR}",
		"PULSE_SERVER": "${localEnv:PULSE_SERVER}",
		"LD_LIBRARY_PATH": "/usr/lib/wsl/lib",
		"LIBGL_ALWAYS_SOFTWARE": "1" // Needed for software rendering of opengl
	},
```

### Repos are not showing up in VS Code source control

This is likely because vscode doesn't necessarily know about other repositories unless you've added them directly. 

```
File->Add Folder To Workspace
```

![Screenshot-26](https://github.com/athackst/vscode_ros2_workspace/assets/6098197/d8711320-2c16-463b-9d67-5bd9314acc7f)


Or you've added them as a git submodule.

![Screenshot-27](https://github.com/athackst/vscode_ros2_workspace/assets/6098197/8ebc9aac-9d70-4b53-aa52-9b5b108dc935)

To add all of the repos in your *.repos file, run the script

```bash
python3 .devcontainer/repos_to_submodules.py
```

or run the task titled `add submodules from .repos`
