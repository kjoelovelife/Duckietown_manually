# Duckietown_manually 

This project is about how to design your own ROS package to have communications with Duckiebot DB21M.
You can check this to see the souce code of [Duckiwtown](https://github.com/duckietown).

## Hardware you need to prepare

Please check this to know: [Duckiebot DB21M](https://get.duckietown.com/collections/dt-robots/products/duckiebot-db21).

If you have it, let's go to the next step!

## 1. Setup operating environment on your host machine.

### 1-1. Install ROS on host machine

Host machine must can use ROS and the version is "Noetic Ninjemys". 

If you don't have any idea to install ROS on your host machine, please check [ROS Installation](http://wiki.ros.org/noetic/Installation) to install ROS.

And these examples below are used on ubuntu 20.04 system. If you use another system, please modify some codes by yourself.

### 1-2. Download the project Duckietown_manually

Please download the project Duckietown_manually through the command below:

```bash=
git clone https://github.com/kjoelovelife/Duckietown_manually
```

![Download the project Duckietown_manually](https://i.imgur.com/nbj5YGr.png)

### 1-3. Build the work space

This project build on some dependencies. Therefore, we need to install these dependencies first. Pleas type the command below to install these dependencies:

```bash=
source ~/Duckietown_manually/DB21M/install/dependencies.sh
```

![Install dependencies](https://i.imgur.com/5ZgFIaP.png)

Awesome! Now we can build the workspace via the command below:

```bash=
cd ~/Duckietown_manually/DB21M/catkin_ws && catkin_make
```

![Build workspace](https://i.imgur.com/Xa6nUF0.png)

### 1-4. Let terminal know where is the project

After building workspace, we must let our terminal know where is the project or we can't use the project! Please type the command below to help terminal know where is the project:

```bash=
echo "source ~/Duckietown_manually/DB21M/setup/environment.sh $duckiebot" >> ~/.bashrc
```

The variable "$duckiebot" is the "hostname" fo your duckiebot. Here is an example, the duckiebot with the hostname "edu", then replace "\$duckiebot" with "edu". After typing the command, we can update terminal to see the information:

![Source workspace](https://i.imgur.com/eIA09iU.png)

Great! Now we have done setting operating environment! We can go to the next step.

## 2. Use keyboard to teleop the duckiebot

### 2-1. Power your duckiebot

Please check the offical document [opmanual_duckiebot-ready-tablet.pdf](https://docs.duckietown.org/daffy/opmanual_duckiebot/opmanual_duckiebot-ready-tablet.pdf) to power and set up your duckiebot.

### 2-2. Launch the launch file 

Now we can type the command below to teleop the duckiebot with keyboard:

```bash=
roslaunch duckiebot_control twist_control_duckiebot.launch
```

![Twist control duckiebot](https://i.imgur.com/aQ1HWKz.png)

Please use the infromation on the terminal to control your duckiebot:

![Use the infromation on the terminal](https://i.imgur.com/McAOqwg.png)

**Notice! If you use the high speed to control the duckiebot, it easily cause the out of power.**

### 2-3. Use rqt_reconfigure to adjust parameter

After teleoping the duckiebot, we can use the rqt plugin - reconfigure to adjust the parameter on the duckiebot. Please type the command below to open the rqt-reconfigure:

```bash=
rosrun rqt_reconfigure rqt_reconfigure
```

![Run the rqt_reconfigure](https://i.imgur.com/IORawk6.png)

And then you can see the window "rqt_reconfigure". Please expand the list on the left side and click the item "inverse_kinematic_reconfigure"

![inverse_kinematic_reconfigure](https://i.imgur.com/PaEhGo4.png)

Great! Now we can adjust the parameters thorugh the sliders.

### 2-4. Save the parameters

After adjusting the parameters, we must save these parameters or will lose them while shutdwon the duckiebot.

Please type the command below to save these parameters you adjsuted:

```bash=
rosservice call /$duckiebot/kinematics_node/save_calibration
```

**Notice! The variable "$duckiebot" should type the hostname of your duckiebot!** Here is an example:

![Save calibration](https://i.imgur.com/ktKX6er.png)

### 3. Shutdown all the ROS programs 

OK! Remember to shutdown all the ROS programs on your host machine.
And now you can use the new parameters to control your duckiebot!