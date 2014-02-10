
% Setup classpath 
jmb_init();

% Init_Node
MASTER_URI='http://localhost:11311';
NODE_NAME='talker';
node=jmb_init_node(NODE_NAME, MASTER_URI);

% Create Publisher
pub=node.newPublisher('/chatter','std_msgs/String');


for i=1:50

	% New a message, setup fields
	msg=org.ros.message.std_msgs.String();
	msg.data=sprintf('Hello World %d', i);

	% Publish 
	pub.publish(msg);

	% ROS_INFO(msg)
	node.getLog().info(['I talked - ' char(msg.data)]);
	pause(1);
end

% Shutdown this node, unregister
node.shutdown()
