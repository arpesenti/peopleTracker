% Setup classpath 
jmb_init();

% Change this to your 
MASTER_URI='http://localhost:11311';
NODE_NAME='listener';
node=jmb_init_node(NODE_NAME, MASTER_URI);

% Create a Subscriber
sub=edu.ucsd.SubscriberAdapter(node,'/chatter','std_msgs/String');

timeout=5;

logger=node.getLog()
while 1

	%Blocking call, return upon timeout or receiving a msg
	msg=sub.takeMessage(timeout);

	if isempty(msg)
		logger.warn('timeout');
	else
		%ROS_INFO(msg)
		logger.info(['I heard - ' char(msg.data)]);
	end
end

% Shutdown this node, unregister
node.shutdown()
