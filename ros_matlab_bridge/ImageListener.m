jmb_init();
MASTER_URI='http://localhost:11311';
NODE_NAME='listener';
node=jmb_init_node(NODE_NAME, MASTER_URI);
sub=edu.ucsd.SubscriberAdapter(node,'/camera/depth/image_rect','sensor_msgs/Image');

timeout=5;

logger=node.getLog()
while 1
	msg=sub.takeMessage(timeout);
	if isempty(msg)
		logger.warn('timeout');
	else
		logger.info(sprintf('I got image %dx%d', msg.widrh, msg.height));
		img=reshape(typecast(msg.data,'single'), msg.width, msg.height);
		imagesc(img);
		drawnow
	end
end

node.shutdown()
