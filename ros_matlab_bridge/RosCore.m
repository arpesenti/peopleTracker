% Setup classpath 
jmb_init()

roscore=org.ros.RosCore.newPublic(11311);
roscore.start();
roscore.awaitStart();
ROS_MASTER_URI=roscore.getUri()
