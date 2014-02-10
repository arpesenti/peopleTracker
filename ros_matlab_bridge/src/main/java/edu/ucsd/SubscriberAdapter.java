package edu.ucsd;
import org.ros.internal.node.DefaultNodeFactory;
import org.ros.message.MessageListener;
import org.ros.message.Message;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Subscriber;
//import org.ros.internal.transport.CircularBlockingQueue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

public class SubscriberAdapter
{
    private java.util.Vector<MxMessageListener> attachedListeners= new java.util.Vector<MxMessageListener>();
	public Subscriber<Message> sub;
	public static boolean showMsg=false;
	public ArrayBlockingQueue<Message> incomingQueue;
	private static final int MESSAGE_BUFFER_CAPACITY=2;
	
    public synchronized void addMxMessageListener(MxMessageListener lis) {
		attachedListeners.addElement(lis);
    }
    public synchronized void removeMxMessageListener(MxMessageListener lis) {
        attachedListeners.removeElement(lis);
    }
    public interface MxMessageListener extends java.util.EventListener {
        public void onNewMessage(java.util.EventObject event);
    }
	public Message takeMessage()  throws InterruptedException
	{
		return takeMessage(5);
	}

	public Message takeMessage(int timeout)  throws InterruptedException
	{
		Message msg=incomingQueue.poll(timeout,TimeUnit.SECONDS);
		if (msg==null)
			System.err.println("error in takeMessage(): no message in the queue in past "+timeout+" seconds");
		return msg;
	}

	public SubscriberAdapter(Node node, String topic, String msg)
	{
		incomingQueue = new ArrayBlockingQueue(MESSAGE_BUFFER_CAPACITY);
		sub=node.newSubscriber(topic, msg);
		sub.addMessageListener(
				new MessageListener<Message>()  {
					public void onNewMessage(Message message) {
						/* put a copy of message into the queue */
						if(!incomingQueue.offer(message) && showMsg)
							System.out.println("java_matlab_bridge: queue_full");

						/* notify listeners */
						/* take a snapshot before walking the concurrent array */
						java.util.Vector<MxMessageListener> attachedListenersSnapshot;
						synchronized(this) {
							attachedListenersSnapshot = (java.util.Vector)attachedListeners.clone();
						}
						for (int i=0; i<attachedListenersSnapshot.size(); i++) {
							attachedListenersSnapshot.elementAt(i).onNewMessage(new java.util.EventObject(message));
						}
					}
				});
		
	}

	public static void main(String[] arg) throws InterruptedException
	{
		Node node = (new DefaultNodeFactory()).newNode(NodeConfiguration.newPrivate().setNodeName("SubscriberAdapter"));
		SubscriberAdapter adp = new SubscriberAdapter(node, "chatter","std_msgs/String");
		adp.addMxMessageListener(
					new MxMessageListener(){
						public void onNewMessage(java.util.EventObject event)
						{
							System.out.println( "Callblack:" + ((org.ros.message.std_msgs.String)event.getSource()).data);
						}
					});
		int i=0;
		while(true)
		{
			Message msg = adp.takeMessage();
			System.out.println( "TakeMessage:" + ((org.ros.message.std_msgs.String)msg).data +i);
			i++;
		}


	}


}
