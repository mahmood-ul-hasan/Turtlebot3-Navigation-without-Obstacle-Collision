# Task
Use ROS, connect with the Gazebo of Turtlebot3, and create a control program that keeps moving the Turtlebot3 in the standard World without hitting obstacles. 

I did this task using two different approaches:
1) Keep Turtlebot3 moving using random pose generated one after one
In the first approach, a random pose goal is generated and Turtlebot3 starts to go to that goal after that another random goal is generated and Turtlebot3 again follows it, this process is continuous in the while loop. The following command is used to run this method 
$ roslaunch keep_turtlebot3_moving move_bot_by_random_goal.launch

2) Keep Turtlebot3 moving using a sequence of pre-defined poses
In the second approach, a sequence of different poses are defined and Turtlebot3 follow these commands one after one, till all pose goal are reached. This method can not move Turtlebot3 continuously all the time. The following command is used to run this method 
$ roslaunch keep_turtlebot3_moving move_bot_by_seq_goal.launch

# Prerequisites
    Ubuntu 20.0.4 recommended
    ROS Noetic
    TurtleBot3

1) Make sure "turtlebot3 navigation," "turtlebot3 simulations," "turtlebot3 slam," and "turtlebot3 teleop" are all installed. If you haven't already installed, Copy the files from https://github.com/ROBOTIS-GIT/turtlebot3 or clone them using the following command
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git)

2) copy the folder "keep_turtlebot3_moving" into catkin_ws/src
$ cd ~/catkin_ws/src/
$ cd ~/catkin_ws/
$ catkin_make

# How To Run
1) I recommend using the following command to avoid getting the model error at each turtlebot3 package node launch:
$ echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc 

2) Run tutlebot3 in gazebo and rviz using following command
$ roslaunch keep_turtlebot3_moving turtlebot3_navigation_gazebo_rviz.launch

3) Set the initial pose of turtlebot3 in rviz
Press the "2D Pose Estimate" button in Rviz, then click in the approximate place where the Turtlebot3 is visible in the Gazebo view and select its orientation before releasing the mouse.

4) Start moving Turtlebot3
launch the move_bot_by_random_goal to keep Turtlebot3 moving by random pose generated one after one using the following command
$ roslaunch keep_turtlebot3_moving move_bot_by_random_goal.launch

launch the move_bot_by_seq_goal to keep Turtlebot3 moving by a sequnence of pre-defined poses using following command
$ roslaunch keep_turtlebot3_moving move_bot_by_seq_goal.launch


# Translated using www.DeepL.com/Translator
ROSを使い、Turtlebot3のGazeboと接続し、障害物にぶつかることなく標準のWorldでTurtlebot3を動かし続ける制御プログラムを作成します。

このタスクは、2種類のアプローチで行いました。
1) 次々と生成されるランダムなポーズを使って Turtlebot3 を動かし続ける
最初のアプローチでは、ランダムなポーズのゴールが生成され、Turtlebot3はそのゴールに向かい始め、次に別のランダムなゴールが生成され、Turtlebot3は再びそれに従います。この方法を実行するには、次のコマンドを使用します。
$ roslaunch keep_turtlebot3_moving move_bot_by_random_goal.launch

2) あらかじめ定義された一連のポーズを使用して Turtlebot3 を移動させ続ける
2つ目のアプローチでは、異なるポーズのシーケンスを定義し、Turtlebot3はすべてのポーズゴールに到達するまで、これらのコマンドに次々と従います。この方法では、Turtlebot3を常に動かし続けることはできません。この方法を実行するには、次のコマンドを使用します。
$ roslaunch keep_turtlebot3_moving move_bot_by_seq_goal.launch

# 前提条件
    Ubuntu 20.0.4推奨
    ROSノエティック
    TurtleBot3

1) "turtlebot3 navigation", "turtlebot3 simulations", "turtlebot3 slam", "turtlebot3 teleop" がすべてインストールされていることを確認する。まだインストールしていない場合は、https://github.com/ROBOTIS-GIT/turtlebot3 からファイルをコピーするか、次のコマンドでクローンしてください。
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git)

2) "keep_turtlebot3_moving "フォルダをcatkin_ws/srcにコピーします。
$ cd ~/catkin_ws/src/
$ cd ~/catkin_ws/
$ catkin_make

# 実行方法
1) turtlebot3 パッケージノード起動のたびにモデルエラーが発生するのを避けるため、以下のコマンドを使用することをお勧めします。
$ echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc 

2) gazebo と rviz で以下のコマンドを使用して tutlebot3 を実行します。
$ roslaunch keep_turtlebot3_moving turtlebot3_navigation_gazebo_rviz.launchを起動します。

3) rviz で turtlebot3 の初期ポーズを設定します。
Rvizの "2D Pose Estimate "ボタンを押し、GazeboビューでTurtlebot3が見えるおおよその場所でクリックし、その方向を選択してからマウスを放します。

4) Turtlebot3の移動を開始します。
move_bot_by_random_goal を起動し、次々と生成されるランダムなポーズで Turtlebot3 を動かし続けるには、次のコマンドを使用します。
$ roslaunch keep_turtlebot3_moving move_bot_by_random_goal.launch

move_bot_by_seq_goal を起動し、あらかじめ定義されたポーズの連続によって Turtlebot3 を移動させるようにするコマンドです。
$ roslaunch keep_turtlebot3_moving move_bot_by_seq_goal.launch



