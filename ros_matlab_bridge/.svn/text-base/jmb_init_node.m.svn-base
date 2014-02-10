% function node=jmb_init_node(NODE_NAME, MASTER_URI)
% This function initialize ROS and craete a ROS_node,
% The code is a translation from rosjava into matlab 
% See http://www.ros.org/wiki/rosjava/Overview/Nodes

function node=jmb_init_node(NODE_NAME, MASTER_URI)
	persistent nodeCache
	if isempty(nodeCache)
		nodeCache=containers.Map();
	elseif ~exist('NODE_NAME')
		%return entire list
		node = nodeCache.keys;
		return
	elseif nodeCache.isKey(NODE_NAME)
		%return corresponding node
		node=nodeCache(NODE_NAME);
		return
	end

	import org.ros.node.*
	import org.ros.internal.node.*
	import org.ros.address.InetAddressFactory;

	%this avoid the log4j error, alternatively, a log4j.xml configuration should be placed in the classpath
	org.apache.log4j.BasicConfigurator.resetConfiguration();
	org.apache.log4j.BasicConfigurator.configure();
	org.apache.log4j.Logger.getRootLogger().setLevel(org.apache.log4j.Level.INFO)

	%ROS_IP is the address of this node
	ROS_IP=InetAddressFactory.newNonLoopback.getHostName();
	nodeConf=NodeConfiguration.newPublic(ROS_IP, java.net.URI(MASTER_URI)); 
	nodeConf.setNodeName(NODE_NAME); %node Name
	nf=DefaultNodeFactory();
	node=nf.newNode(nodeConf);


	nodeCache(NODE_NAME) = node;
