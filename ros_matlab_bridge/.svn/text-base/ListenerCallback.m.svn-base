%matlab callback made with
%http://undocumentedmatlab.com/blog/matlab-callbacks-for-java-events/

% Setup classpath 
jmb_init();

MASTER_URI='http://localhost:11311';
NODE_NAME='listener';
node=jmb_init_node(NODE_NAME, MASTER_URI);

% Create a Subscriber
sub=edu.ucsd.SubscriberAdapter(node,'/chatter','std_msgs/String');

% Callback function, note the function prototype (handle,evt). When called, extract msg by msg=evt.getSource();
callback = @(handle, evt) node.getLog.info(['I heard - ' char(evt.getSource().data)]);

% Hookup the callback function
set(sub, 'OnNewMessageCallBack', callback)
