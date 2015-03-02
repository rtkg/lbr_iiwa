package application;

import java.io.IOException;
import java.util.concurrent.Semaphore;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import sun.awt.Mutex;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;

public class JointTargetsListner extends Thread {
	private String hostname;
	int SIZE = 7;
	private Client mylink_send, mylink_recv;
	private boolean quit;
	private JointPosition jp;
	private double joint_targets[], joint_states[];
	private static final Lock lockObj = new ReentrantLock();
	private static final Condition cond = lockObj.newCondition();
	//private final Semaphore sem;
	
	public JointPosition getJP(JointPosition js) {//  throws InterruptedException {
		lockObj.lock();
		JointPosition jp = new JointPosition(joint_targets);
		for(int j=0; j<SIZE; ++j) {
			joint_states[j] = js.get(j);
		}
		//signal on condition
		//cond.signal();
		lockObj.unlock();
		
		//System.out.println("released mutex");
		return jp;
	}
	
	public synchronized void setStop() {
		quit = true;
	}
	
	public JointTargetsListner() {	
		//sem = new Semaphore(1);
        hostname = "172.31.1.3";
        joint_targets = new double[SIZE];
        joint_states = new double[SIZE];
        for(int i=0; i<SIZE; i++) joint_targets[i] = 0;
        jp = new JointPosition(joint_targets);
        try {
			mylink_send = new Client(5011, 2, hostname, 1);
			mylink_recv = new Client(5010, 1, hostname, 1);

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        quit = false;
        
	}
	
	public void run() {
		
		double jt_buff[] = new double[SIZE];
		double js_buff[] = new double[SIZE];
		
		while (mylink_send.isActive() && !quit) {
			
			try {
				//non-blocking
				lockObj.lock();
				//wait on condition
				//cond.await();
				//copy joint_states to buffer
				for(int i=0; i<SIZE; i++) {
					js_buff[i] = joint_states[i];
				}
				lockObj.unlock();
				
				mylink_send.RecvDoubles(jt_buff, SIZE);
				mylink_recv.SendDoubles(js_buff, SIZE);
				
				lockObj.lock();
				//copy buffer to joint_targets
				for(int i=0; i<SIZE; i++) {
					joint_targets[i] = jt_buff[i];
				}
				lockObj.unlock();
				
				//blocking
				/*
				lockObj.lock();
				//wait on condition
				cond.await();
				//copy joint_states to buffer
				
				mylink_send.RecvDoubles(joint_targets, SIZE);
				mylink_recv.SendDoubles(joint_states, SIZE);
				
				lockObj.unlock();
				*/
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				return;
			} 
			/*catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}*/
			
			
			
			
			
				
		}
		
	}
}
