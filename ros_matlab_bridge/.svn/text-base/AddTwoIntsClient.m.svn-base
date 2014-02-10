% Prerequisites
% run the python add_two_ints_server by
% rosrun test_ros add_two_ints_server
% DO NOT run the add_two_ints_server in rospp/rospy_tutorials, they use different service .srv files

% Setup classpath 
jmb_init();

% Init_Node
MASTER_URI='http://localhost:11311';
NODE_NAME='add_two_ints_client';
node=jmb_init_node(NODE_NAME, MASTER_URI);



% Service Call Instance
client=node.newServiceClient('add_two_ints','test_ros/AddTwoInts');

%create a request instance, 
srv=org.ros.service.test_ros.AddTwoInts()
req=srv.createRequest();
req.a = 100;
req.b = 200;

%call server and parse response
adapter = edu.ucsd.ServiceResponseAdapter;
client.call(req, adapter);
resp=adapter.waitForResponse();
disp(resp.sum)
