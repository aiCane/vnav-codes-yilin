#import "@preview/charged-ieee:0.1.4": ieee

#show: ieee.with(
  title: [Lab 3 Report: EXERCISES],
  authors: (
    (
      name: "Yilin Zhang, 23020036094, Group 31",
      department: [School of Computer Science],
      organization: [Ocean University of China],
      location: [Qingdao, China],
      email: "zyl8820@stu.ouc.edu.cn"
    ),
  ),
  // bibliography: bibliography("refs.yml"),
  figure-supplement: [Fig.],
)

#let appendix(body) = {
  set heading(numbering: "A.1", supplement: [Appendix])
  counter(heading).update(0)
  show heading.where(level: 1): it => {
    set text(size: 11pt, weight: "medium")
    set align(center)
    show: smallcaps
    v(1em)
    [#it.supplement #counter(heading).display() \ ]
    it.body
  }
  body
}

// Display block code in a larger block
// with more padding.
#show raw.where(block: true): block.with(
  width: 100%,
  fill: luma(240),
  inset: 10pt,
  radius: 4pt,
)

#show link: underline
#set math.mat(delim: "[")
#set math.vec(delim: "[")

= Individual

== Deliverable - Transformations in Practice

#text(size: 0.8em)[_A._ MESSAGE VS. TF]

It says: "Based on the documentation for #link("http://docs.ros.org/melodic/api/tf2/html/index.html")[tf2] and its package listing #link("http://wiki.ros.org/tf2")[page], answer the following questions in a text file called `deliverable_1.txt`." @VNAV Well, I will put the questions to @a_deliverable_transformations_in_practice, and post my answers below.

`1` Assume we have the premise code below:

```cpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
tf2::Quaternion quat_tf;
geometry_msgs::Quaternion quat_msg = ...;
```

I fond three methods on #link("https://wiki.ros.org/tf2/Tutorials/Quaternions")[Quaternion Basics]:

```cpp
tf2::convert(quat_msg , quat_tf);
```
```cpp
tf2::fromMsg(quat_msg, quat_tf);
```
or for the other conversion direction, `tf2::toMsg`.
```cpp
quat_msg = tf2::toMsg(quat_tf);
```

`2` Assume we have the premise code below:

```cpp
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
tf2::Quaternion quat_tf;
quat_tf.setRPY(1.0, 0.0, 0.0);
geometry_msgs::Quaternion quat_msg;
```

We may use this method. #footnote[According to #link("https://wiki.ros.org/tf2/Tutorials/Migration/DataConversions")[Converting Datatypes], `tf2/transform_datatypes.h` doesn't have the quaternion helper functions (you should use `tf2::Quaternion` methods in conjunction with `tf2::convert()`)]

```cpp
tf2::convert(quat_tf, quat_msg);
```

`3` I would use `getW()` in Quaternion class.#footnote[See #link("https://docs.ros.org/en/kinetic/api/tf2/html/classtf2_1_1Quaternion.html")[member function documentation] of `tf2::Quaternion Class` or the 346 line in file #link("https://docs.ros.org/en/kinetic/api/tf2/html/Quaternion_8h_source.html")[`Quaternion.h`].]

```cpp
TF2SIMD_FORCE_INLINE const tf2Scalar& getW() const { return m_floats[3]; }
```

#text(size: 0.8em)[_B._ CONVERSION]

The following questions will prepare me for converting between different rotation representations in C++ and ROS. @VNAV

`1` This single function call is discribed in #link("https://docs.ros.org/en/humble/p/tf2/generated/function_namespacetf2_1a4ba73137cbca3cb04ae2072075dc7469.html")[tf2: Humble documentation]. Here `a` is the object to get data from (it represents a rotation/quaternion).

```cpp
template<class A>
double tf2::getYaw(const A &a)
```

`2` We need two function calls to implement this: `tf2::fromMsg` #footnote[#link("https://docs.ros.org/en/jade/api/tf2_eigen/html/tf2__eigen_8h.html")[199-201 lines from `tf2_eigen.h`]] and `toRotationMatrix()`.

```cpp
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>

// Assume we have an Eigen Quaternion container
Eigen::Quaterniond eigen_quat;

// We use
tf2::fromMsg(quat_msg, eigen_quat);
Eigen::Matrix3d rotation_matrix = eigen_quat.toRotationMatrix();
```

== Deliverable - Modelling and control of UAVs

#text(size: 0.8em)[_A._ STRUCTURE OF QUADROTORS]

Set that we have the crossing x-axis, and y-axis, shown in @my_drone. The length from each wing to x-axis or y-axis are set to $L$. Assume all the wings _1_ to _4_ has the same lift force, and at origen, the drone is horizontal.

#figure(
  image("figures/my_drone.jpg", width: 80%),
  placement: auto,
  caption: [My hand draw (qaq), with length $L$ and coordinate system depicted.],
) <my_drone>

We have the same yaw orientation $Psi$ for quadrotor _(a)_. To make it simple, the value are all set to $1$.

$
  Psi = mat(
    1, 1, 1, 1
  ).
$

For the x-axis, due to the amperes right handed screw rule, wing _1_ and _2_ are positive, while wing _3_ and _4_ are negative. For example, $tau_{x 1} = F times d = 1 times L = L$. Therefore,

$
  x = mat(
    L, L, -L, -L
  ).
$

Similarly, for the y-axis, we have

$
  y = mat(
    L, -L, -L, L
  ).
$

Since wing _1_ and _3_ spin counter clockwise, they generate a torque with a upper direction. While wing _2_ and _4_ spin clockwise, they are just the opposite. We cannot calculate the exact torque generated by each wing, so here we use $c$ s.t. $tau_o = c dot tau_q$, where $tau_q$ refers to the torque generated by any single wing, and $tau_o$ is the influence of the quadrotor.

$
  z = mat(
    -c, c, -c, c
  ).
$

Since we got yaw orientation $Psi$ and position $[x,y,z]$, we can write down the $F$ matrix of quadrotor _(a)_:

$
  F_a = mat(
    1, 1, 1, 1;
    L, L, -L, -L;
    L, -L, -L, L;
    -c, c, -c, c
  ).
$

For quadrotor _(b)_, the rotation direction of wing _3_ and _4_ are opposite to the ones of quadrotor _(a)_. Similarly, we can write down the $F$ matrix of quadrotor _(b)_:

$
  F_b = mat(
    1, 1, 1, 1;
    L, L, -L, -L;
    L, -L, -L, L;
    -c, c, c, -c
  ).
$

You can see that only position $z$ is different. Through calculation in @rank_a and @rank_b, we get rank of both matrices. @rank

$
  cases(
    "rank_a" = 4,
    "rank_b" = 3
  )
$

#text(size: 0.8em)[_B._ CONTROL OF QUADROTORS]

The standard position controller computes a desired net external force $F_"des"$ (expressed in the inertial frame) to achieve the desired acceleration. However, the *true dynamics* of the system are:

$
  m dot.double(p) = underbrace(f R e_3, "thrust") - m g e_3 + underbrace(F_"drag"^I}, "drag (inertial frame)").
$

To ensure the actual acceleration $dot.double(p)$ matches the controller's desired acceleration, the thrust $f R e_3$ must not only supply $F_"des"$ but also counteract the drag force. Therefore, $F_"des"$ must be adjusted accordingly.

From onboard sensors we get all $v_x$, $v_y$, $v_z$, thus calculate $(v^b)^2 = vec(-v^b_x abs(v^b_x), -v^b_y abs(v^b_y), -v^b_z abs(v^b_z))$.

$
  F_"adj" = F_"des" - F_"drag"^I.
$

= Team

Follow appendix @a_trajectory_tracking_for_uavs and @a_launching_the_tesse_simulator_with_ros_bridge for the set up.

== Deliverable - Geometric controller for the UAV

The implementation is strongly based on the publication @Lee2010Geometric. A lot of functions can be find as equations here. The whole job is divided into six parts, with strongly recommended suggestions in each part. Yet the introduction also says "but not mandatory if you have better alternatives in mind:)". Awsome... Basicly, we are implementing a controller node called `controllerNode`, which is a class. The class has its field, a constructor, mothods `onDesiredState()`, `onCurrentState()`, and `controlLoop()`.

_Part 1_ was put in the declaration field. Here, we were asked to declare two subscribers, one publisher and a timer. This part had nothing special. I would put all my code in the appendices.

In _part 2_, we focused on the constructor, and initialized our handlers from _part 1_. Specifically bind `controllerNode::onDesiredState()` to the topic "desired_state", bind `controllerNode::onCurrentState()` to the topic "current_state", and bind `controllerNode::controlLoop()` to the created timer, at frequency given by the "hz" variable. Noticed a `ROS::nodeHandle` `nh` variable was already available as a class member, `nh.subscribe()` was the method chosen for the subscribers.

_Part 3_ was all about method `onDesiredState()`. We filled xd, vd, ad and yawd using "v << vx, vy, vz;" to fill a vector with Eigen.

_Part 4_ continued with the method `onCurrentState()`. The job here was to fill the current state variables x, v, R, and omega from the incoming odometry message. For position and linear velocity, it was straightforward to copy from `cur_state.pose.pose.position` and `cur_state.twist.twist.linear`. For the orientation matrix R, we had to convert the quaternion in the message to a rotation matrix. I used `tf2::Matrix3x3` to do that. The tricky part was the angular velocity omega. The message gave it in the world frame, but the paper and our code needed it in the body frame. So we transformed it by multiplying with the transpose of R.

_Part 5_ was the biggest part, where we actually implemented the geometric controller in the `controlLoop()` method. It followed the paper's equations step by step. First, we computed the position and velocity errors ex and ev. Then we computed the desired force vector F_des, and from that, we got the desired body z-axis b3d. Using the desired yaw, we constructed a desired body x-axis b1d, and then used cross products to get a right-handed coordinate system for the desired orientation matrix Rd. After that, we computed the orientation error er and the angular velocity error eomega. The next step was to compute the total thrust force f and the control moment M. Finally, we needed to convert this desired wrench (force and moment) into actual propeller speeds. This involved solving a linear system using the F2W matrix we built in the constructor, and then taking the signed square root of the result. The last bit was to pack these four rotor speeds into a ROS message and publish it.

_Part 7_ was not listed in the original instructions, but it was in the code. It was about building the F2W matrix. This matrix maps the four rotor speed squares to the total thrust and three body moments. Its coefficients came from the geometry of the quadrotor (arm length d) and the aerodynamic coefficients cf and cd. Basically, it summed up how much force and torque each propeller contributed. We computed a `d_by_sqrt2` term for convenience and then filled the 4x4 matrix. Without initializing this matrix, the solver in part 5 would have nothing to work with and crash.

That sums up the implementation. The rest was just the main function to start the node and let ROS spin.

Then we comes to _Part 6_, which was about tuning the controller gains. The gains kx, kv, kr, komega were loaded from the ROS parameter server in the constructor. This way, we could change them easily using a launch file without recompiling the code. The constructor printed them out with `ROS_INFO` so we could check. I tried "4, 4, 8, 4" as the initial values, surprisingly, it worked very well. Therefore, I just need to fine-tune them a bit. "kx" means how much the drone wants to go towards the desired position. "kv" means how much the drone wants to match the desired velocity. "kr" means how much the drone wants to go correct its posture errors. "komega" means how much the drone wants to match the desired angular velocity. In this lab's situation, the desired drone was simple. There is no need to bahave presicely, "kx" and "komega" could be lower. And on the opposite, "kv" and "kr" coule be a little bit higher. After a 6 hours of tuning, I got "3.75, 4.5, 10.0, 3.75".

I got a video showing the quadrotor completing one round of the circular trajectory in `rviz`, and put the link on my Google Drive #link("https://drive.google.com/file/d/1M4gUiEtQukBI5kzaVriR1AQiGOibS6ME/view?usp=share_link")[here].

= Some Words

This lab was terrible. I spent 3 weeks just to encourage myself, I have no time to waste and I need to start somewhere. Thanks for the delay of submission, I was able to finish it in time. Facing the deadline staring at me next Tuesday, I finally start to do the team part this Wednesday. All I got was a bunch of code that I had to understand and modify and a publicated paper. I hate reading papers, I cannot understand a word! It's horrible!

Anyway, also in this lab report, I learned a new paper writing language `typst`. And this report was my first homework submitted using `typst`. This language is definitely new and fresh, with much easier grammer than `latex`. It has some little problems, such as no `#thanks` option, no `#appendix` nor `#appendices`. I managed to deal with them, searching for alternatives and finding a way to work around them. And I got this, as you can see. It is hard to tell if it worth to replace `typst` from `latex`, but `typst` definately worth a try.

#bibliography("refs.yml")

#show: appendix

// Display inline code in a small box
// that retains the correct baseline.
#show raw.where(block: false): box.with(
  fill: luma(240),
  inset: (x: 3pt, y: 0pt),
  outset: (y: 3pt),
  radius: 2pt,
)

// Reset raw blocks to the same size as normal text,
// but keep inline raw at the reduced size.
#show raw.where(block: true): set text(0.8em)

= Individual

== Deliverable - Transformations in Practice
<a_deliverable_transformations_in_practice>

#text(0.8em)[_A._ MESSAGE VS. TF]

`1` Assume we have an incoming `geometry_msgs::Quaternion quat_msg` that holds the pose of our robot. We need to save it in an already defined `tf2::Quaternion quat\_tf` for further calculations. Write one line of C++ code to accomplish this task.

`2` Assume we have just estimated our robot’s newest rotation and it’s saved in a variable called `quat_tf` of type `tf2::Quaternion`. Write one line of C++ code to convert it to a `geometry_msgs::Quaternion` type. Use `quat_msg` as the name of the new variable.

`3` If you just want to know the scalar value of a `tf2::Quaternion`, what member function will you use?

#text(0.9em)[_B._ CONVERSION]

`1` Assume you have a `tf2::Quaternion quat_t`. How to extract the yaw component of the rotation with just one function call?

`2` Assume you have a `geometry_msgs::Quaternion quat_msg`. How to you convert it to an Eigen 3-by-3 matrix? Refer to #link("http://docs.ros.org/jade/api/tf2_eigen/html/index.html")[this] and #link("http://eigen.tuxfamily.org/dox-3.2/classEigen_1_1Quaternion.html")[this] for possible functions. You probably need two function calls for this.

== Deliverable - Modelling and control of UAVs

#text(0.8em)[_A._ STRUCTURE OF QUADROTORS]

#figure(
  image("figures/drone_spinning.png", width: 80%),
)

The figure above depicts two quadrotors _(a)_ and _(b)_. Quadrotor _(a)_ is a fully functional UAV, while for Quadrotor _(b)_ someone changed propellers 3 and 4 and reversed their respective rotation directions.

Show mathematically that quadrotor _(b)_ is not able to track a trajectory defined in position $[x,y,z]$ and yaw orientation $Psi$.

$
  F_a = mat(
    1, 1, 1, 1;
    L, L, -L, -L;
    L, -L, -L, L;
    -c, c, -c, c
  ) \
  arrow.r.double mat(
    1, 1, 1, 1;
    1, 1, -1, -1;
    1, -1, -1, 1;
    -1, 1, -1, 1
  ) \
  arrow.r.double mat(
    1, 1, 1, 1;
    0, 0, -2, -2;
    0, -2, -2, 0;
    0, 2, 0, 2
  ) \
  arrow.r.double mat(
    1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1
  ).
$ <rank_a>

$
  F_b = mat(
    1, 1, 1, 1;
    L, L, -L, -L;
    L, -L, -L, L;
    -c, c, c, -c
  ) \
  arrow.r.double mat(
    1, 1, 1, 1;
    1, 1, -1, -1;
    1, -1, -1, 1;
    -1, 1, 1, -1
  ) \
  arrow.r.double mat(
    1, 1, 1, 1;
    0, 0, -2, -2;
    0, -2, -2, 0;
    0, 2, 2, 0
  ) \
  arrow.r.double mat(
    1, 1, 0, 0;
    0, 1, 1, 0;
    0, 0, 1, 1;
    0, 0, 0, 0
  )
$ <rank_b>

#text(0.8em)[_B._ CONTROL OF QUADROTORS]

Assume that empirical data suggest you can approximate the drag force (in the body frame) of a quadrotor body as:

$
F^b = mat(
  0.1, 0, 0;
  0, 0.1, 0;
  0, 0, 0.2;
) (v^b)^2
$

With $(v^b)^2 = [-v^b_x abs(v^b_x), -v^b_y abs(v^b_y), -v^b_z abs(v^b_z)]^top$, and $v_x$, $v_y$, $v_z$, being the quadrotor velocities along the axes of the body frame.

With the controller discussed in class (see referenced paper @Lee2010Geometric), describe how you could use the information above to improve the tracking performance.

= Team

== Trajectory tracking for UAVs
<a_trajectory_tracking_for_uavs>

`GETTING THE CODEBASE`

```bash
sudo apt install ros-noetic-ackermann-msgs
```
```bash
cd ~/labs
git pull
```
```bash
cp -r ~/labs/lab3/. ~/vnav_ws/src
cd ~/vnav_ws
```
```bash
cd ~/vnav_ws/src/tesse-ros-bridge/tesse-interface
pip install -r requirements.txt  # Dependencies
pip install .
```
```bash
cd ~/vnav_ws/src && git clone https://github.com/ethz-asl/mav_comm.git
```
```bash
catkin build
source devel/setup.bash
```
```bash
cd ~/vnav-builds/lab3/
chmod +x lab3.x86_64
```

== Launching the TESSE simulator with ROS bridge
<a_launching_the_tesse_simulator_with_ros_bridge>

```bash
cd ~/vnav-builds/lab3/
./lab3.x86_64
```
```bash
roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch
```
```bash
cd ~/vnav_ws/src/controller_pkg
rviz -d rviz/lab3.rviz
```

== Deliverable - Geometric controller for the UAV (50 pts)

After reading the reference @Lee2010Geometric - _thoroughly_ - get together with your teammates and open the source file `controller_node.cpp`. In this file, you will find detailed instructions to fill in the missing parts and make your quadrotor fly! To have full credits for this part of the lab, your team needs to complete the following:

- Implement all missing parts in `controller_node.cpp` and ensure everything compiles.
- Tune parameters so that the quadrotor achieves stable circular trajectory.
- A video showing the quadrotor completing one round of the circular trajectory in `rviz` (can either be a screen capture, or a video shot using your phone). Please upload the video onto either Google drive or Dropbox, generate a publicly viewable link, and put the link in a text file called `rviz_drone_flight.txt` in your repo.

```bash
catkin build # from ~/vnav_ws
```
```bash
roslaunch controller_pkg traj_tracking.launch
```

#figure(
  image("figures/yeah_drone.png", width: 80%),
  placement: auto,
  scope: "parent",
) <yeah_drone>

=== `controller_node.cpp`

```cpp
// PART 1
ros::Subscriber des_state_sub, cur_state_sub;
ros::Publisher propeller_speeds_pub;
ros::Timer control_timer;
```

```cpp
// PART 2
des_state_sub = nh.subscribe("desired_state", 1024, &controllerNode::onDesiredState, this);
cur_state_sub = nh.subscribe("current_state", 1024, &controllerNode::onCurrentState, this);
propeller_speeds_pub = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1024);
control_timer = nh.createTimer(ros::Duration(1.0/hz), &controllerNode::controlLoop, this);
```

```cpp
// PART 3
// 3.1
xd << des_state.transforms[0].translation.x,
      des_state.transforms[0].translation.y,
      des_state.transforms[0].translation.z;

vd << des_state.velocities[0].linear.x,
      des_state.velocities[0].linear.y,
      des_state.velocities[0].linear.z;

ad << des_state.accelerations[0].linear.x,
      des_state.accelerations[0].linear.y,
      des_state.accelerations[0].linear.z;
```
```cpp
// PART 3
// 3.2
tf2::Quaternion quat;
tf2::fromMsg(des_state.transforms[0].rotation, quat);
yawd = tf2::getYaw(quat);
```

```cpp
// PART 4
// Position
x << cur_state.pose.pose.position.x,
    cur_state.pose.pose.position.y,
    cur_state.pose.pose.position.z;

// Velocity
v << cur_state.twist.twist.linear.x,
    cur_state.twist.twist.linear.y,
    cur_state.twist.twist.linear.z;

// Orientation
tf2::Quaternion quat;
tf2::fromMsg(cur_state.pose.pose.orientation, quat);

tf2::Matrix3x3 tfm(quat);
for (int i = 0; i < 3; i++) {
  for (int j = 0; j < 3; j++) {
    R(i,j) = tfm[i][j];
  }
}

// Angular velocity
Eigen::Vector3d omega_world;
omega_world << cur_state.twist.twist.angular.x,
              cur_state.twist.twist.angular.y,
              cur_state.twist.twist.angular.z;
omega = R.transpose() * omega_world;
```

```cpp
// PART 5
// 5.1
ex = x - xd;
ev = v - vd;
```

```cpp
// PART 5
// 5.2
Eigen::Vector3d F_des = -kx*ex - kv*ev + m*g*e3 + m*ad;
Eigen::Vector3d b3d = F_des.normalized();
Eigen::Vector3d b1d_desired(cos(yawd), sin(yawd), 0);

Eigen::Vector3d b2d = (b3d.cross(b1d_desired)).normalized();
Eigen::Vector3d b1d = (b2d.cross(b3d)).normalized();

Eigen::Matrix3d Rd;
Rd.col(0) = b1d;
Rd.col(1) = b2d;
Rd.col(2) = b3d;
```

```cpp
// PART 5
// 5.3
er = Vee(0.5 * (Rd.transpose() * R - R.transpose() * Rd));
eomega = omega;
```

```cpp
// PART 5
// 5.4
double f = F_des.dot(R * e3);
Eigen::Vector3d M = -kr*er - komega*eomega + omega.cross(J * omega);
```

```cpp
// PART 5
// 5.5
Eigen::Vector4d W;
W << f, M.x(), M.y(), M.z();

Eigen::Vector4d omega_sq = F2W.colPivHouseholderQr().solve(W);

Eigen::Vector4d rotor_speeds;
for (int i = 0; i < 4; i++) {
  rotor_speeds(i) = signed_sqrt(omega_sq[i]);
}
```

```cpp
// PART 5
// 5.6
mav_msgs::Actuators control_msg;
control_msg.angular_velocities.resize(4);
for (int i = 0; i < 4; i++) {
  control_msg.angular_velocities[i] = rotor_speeds(i);
}
propeller_speeds_pub.publish(control_msg);
```

```cpp
// PART 7
double d_by_sqrt2 = d/std::sqrt(2.0);
F2W <<
    cf,            cf,            cf,            cf,
    cf*d_by_sqrt2, cf*d_by_sqrt2,-cf*d_by_sqrt2,-cf*d_by_sqrt2,
   -cf*d_by_sqrt2, cf*d_by_sqrt2, cf*d_by_sqrt2,-cf*d_by_sqrt2,
    cd,           -cd,            cd,           -cd;
```

```launch
<!-- Controller gains -->
<param name="kx" type="double" value="3.75">
<param name="kv" type="double" value="4.5">
<param name="kr" type="double" value="10.0">
<param name="komega" type="double" value="3.75">
```
