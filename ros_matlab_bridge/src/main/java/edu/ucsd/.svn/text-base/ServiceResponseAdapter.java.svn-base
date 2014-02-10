package edu.ucsd;
import org.ros.node.service.*;
import org.ros.exception.RemoteException;
import java.util.concurrent.*;

public class ServiceResponseAdapter<MessageType> implements ServiceResponseListener<MessageType>
{
	public MessageType lastResponse=null;
	public Semaphore semResponse = new Semaphore(0);

	/**
	 * This function returns when answer comes in.
	 */ 
	public MessageType waitForResponse() throws InterruptedException
	{
		return waitForResponse(5);
	}
	public MessageType waitForResponse(int timeout) throws InterruptedException
	{
		if(!semResponse.tryAcquire(timeout, TimeUnit.SECONDS)){
			lastResponse=null;
			System.err.println("Timeout waiting for service");
		}
		return lastResponse;
	}
	
	public void onSuccess(MessageType response)
	{
		semResponse.drainPermits();
		lastResponse=response;
		semResponse.release();
	}

	public void  onFailure(RemoteException e)
	{
		System.out.println("error");
		lastResponse=null;
	}
}
