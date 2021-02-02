# Maskon

**Maskon** is a surveillance setup built as a ROS package for mall-like environments in the times like COVID-19.  It includes checking people for the mask at the entrance and also a bot that roams in the mall checking people for the mask. 

* [Environment](#Environment)
* [Approach](#Approach)
* [Installation](#Installation)
* [Usage](#Usage)
* [Results](#Results)
* [Team](#Team)

## Environment

We used the Gazebo simulator to simulate a mall-like environment and created different realistic human models using [Make Human](http://www.makehumancommunity.org/) software with and without a mask. And we used [turtlebot](http://wiki.ros.org/turtlebot3) as our surveillance bot. Turtlebot contains a lidar mounted on it and we have mounted a camera on it. 

<div align="center"><img src="assets/mall.jpg" width="80%"/><p><i>Mall-like environment</i></p><img src="assets/bot.png" width="50%"/><p><i>Turtlebot</i></p><img src="assets/diff_humans.jpg" width="80%"/><p><i>Different Human Models</i></p></div>

## Approach

We have two different scenes here:

* **Entrance checking node :** We will have a camera at the entrance of the mall. So for every frame, we have detected faces in that frame using the [MTCNN](https://pypi.org/project/mtcnn/) library then classify those faces whether it has a mask on it or not using a **CNN** model which we trained on a dataset that contains faces with and without masks.
* **Bot node :** The bot *roams* inside the mall by avoiding obstacles using the data from the lidar sensor mounted on it. We will have a camera on the bot. So for every frame, we have extracted human poses using the [posenet](https://github.com/rwightman/posenet-python) library then extracted human faces from the pose information. And then classify those faces whether it has a mask on it or not using a **CNN** model which we trained on a dataset that contains faces with and without masks. 

## Installation

1. Firstly install ROS-Neotic, you can follow the instructions from the official [website](http://wiki.ros.org/noetic/Installation/Ubuntu#Installation).

2. Install catkin tools for python3

   ```shell
   sudo apt install python3-catkin-tools
   ```

3. Create a catkin workspace if you don't have one

   ```shell
   mkdir -p ~/maskon_ws/src
   cd ~/maskon_ws
   catkin init
   catkin_make
   ```

   then the `devel/setup.bash` file of your workspace must be sourced every time when you want to use ROS packages created inside this workspace.

   ```shell
   gedit ~/.bashrc
   source ~/maskon_ws/devel/setup.bash
   ```

4. As this package depends on turtlebot3 you need to clone turtlebot3 packages  into the `maskon_ws/src` directory

   ```shell
   cd ~/maskon_ws/src
   git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
   git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
   git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
   cd ~/maskon_ws 
   catkin_make
   ```

5. Clone this repository into the `maskon_ws/src` directory

   ```shell
   cd ~/maskon_ws/src
   git clone https://github.com/Robotics-Club-IIT-BHU/Maskon.git
   cd ~/maskon_ws 
   catkin_make
   ```

6. We need to download `worlds` directory as we didn't include it in this repository due to its size

   ```shell
   cd ~/maskon_ws/src/Maskon
   wget https://github.com/Robotics-Club-IIT-BHU/Maskon/releases/download/v1.0/worlds.zip
   unzip worlds.zip
   ```
   
7. Install required python libraries 

   ```shell
   cd ~/maskon_ws/src/Maskon
   pip3 install -r requirements.txt 
   ```

## Usage

There are two launch files for [entrance check](launch/enter_mall.launch) and [inside mall](launch/in_mall.launch):

1. Now launch `enter_mall.launch` file

   ```shell
   roslaunch Maskon enter_mall.launch
   ```

   you can see the gazebo window and Image window pops up as shown

   <div align="center"><img src="assets/enter_mall.png"/></div>

2. Now launch `in_mal.launch` file

   ```shell
   roslaunch Maskon in_mall.launch
   ```

   you can see the gazebo window and Bot's View window pops up as shown

<div align="center"><img src="assets/in_mall.png"/></div>

## Results

| <div align="center"><img src="assets/no_mask.gif"/></div>    | <div align="center"><img src="assets/with_mask.gif"/></div>  |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| <div align="center">You can see a person without mask and it draws a red box around his face</div> | <div align="center">You can see a person with mask and it draws a green box around his face</div> |

## Team

#### Makers

<table>
   <td align="center">
      <a href="https://github.com/ananya130">
         <img src="https://avatars.githubusercontent.com/u/73734544?s=400&v=4" width="100px;" alt=""/>
         <br />
         <sub>
            <b>Ananya Singh</b>
         </sub>
      </a>
      <br />
   </td>
   <td align="center">
      <a href="https://github.com/jatin1604">
         <img src="https://avatars.githubusercontent.com/u/60649790?s=400&v=4" width="100px;" alt=""/>
         <br />
         <sub>
            <b>Eaga Jatin</b>
         </sub>
      </a>
      <br />
   </td>
   <td align="center">
      <a href="https://github.com/payal116">
         <img src="https://avatars.githubusercontent.com/u/60649731?s=400&v=4" width="100px;" alt=""/>
         <br />
         <sub>
            <b>Payal Pote</b>
         </sub>
      </a>
      <br />
   </td>
   <td align="center">
      <a href="https://github.com/vstark21">
         <img src="https://avatars.githubusercontent.com/u/67263028?s=400&u=b29c82a39fa1df1689fbb65d8dc95a11c45d4d0e&v=4" width="100px;" alt=""/>
         <br />
         <sub>
            <b>Vishwas Chepuri</b>
         </sub>
      </a>
      <br />
   </td>
</table>

#### Mentors

<table>
   <td align="center">
      <a href="https://github.com/lok-i">
         <img src="https://avatars1.githubusercontent.com/u/54435909?s=400&u=29af076049dab351b2e43621e9a433919bf50fb1&v=4" width="100px;" alt=""/>
         <br />
         <sub>
            <b>Lokesh Krishna</b>
         </sub>
      </a>
      <br />
   </td>
   <td align="center">
      <a href="https://github.com/NiranthS">
         <img src="https://avatars3.githubusercontent.com/u/44475481?s=400&v=4" width="100px;" alt=""/>
         <br />
         <sub>
            <b>Niranth Sai</b>
         </sub>
      </a>
      <br />
   </td>
</table>
