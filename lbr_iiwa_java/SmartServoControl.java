package application;


import com.kuka.common.StatisticTimer;
import com.kuka.common.ThreadUtil;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.motionModel.DirectServo;
import com.kuka.roboticsAPI.motionModel.IDirectServoRuntime;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class SmartServoControl extends RoboticsAPIApplication {
	  // members
    private LBR _theLbr;
    /**
     * Will be initialized from the routine "createTool()".
     * 
     * IMPORTANT NOTE: Set the load mass properly
     */
    private IDirectServoRuntime theSmartServoRuntime = null;
    private int _count = 0;
   // private JointStreamer js;
    private JointTargetsListner jt;
    
    @Override
    public void initialize()
    {
        getContext().dumpDevices();

        // Locate the "first" Lightweight Robot in the system
        _theLbr = ServoMotionUtilities.locateLBR(getContext());
        //js = new JointStreamer();
        jt = new JointTargetsListner();
    }

    /**
     * Move to an initial Position WARNING: MAKE SHURE, THAT the pose is
     * collision free.
     */
    public void moveToInitialPosition()
    {
        _theLbr.move(
                ptp(0., 0., 0., 0, 0.,
                        0., 0.).setJointVelocityRel(0.1));
        
        /*
         * 
         * For Completeness Sake, the validation is performed here Even it would
         * not be necessary within this sample.
         * 
         * As long, as you'd remain within position control, the validation is
         * not necessary ... but, lightweight robot without ImpedanceControl is
         * a car without fuel...
         * 
         * Note: The Validation itself justifies, that in this very time
         * instance, the load parameter setting was sufficient. This does not
         * mean by far, that the parameter setting is valid in the sequel or
         * lifetime of this program
     
        try
        {
            if (!SmartServo.validateForImpedanceMode(_toolAttachedToLBR))
            {
                System.out
                        .println("Validation of Torque Model failed - correct your mass property settings");
                System.out
                        .println("SmartServo will be available for position controlled mode only, until validation is performed");
            }
        }
        catch (IllegalStateException e)
        {
            System.out.println("Omitting validation failure for this sample\n"
                    + e.getMessage());
        }
            */
    }

    // Sleep in between
    //private int _milliSleepToEmulateComputationalEffort = 10;//000;
    //private int _numRuns = 10000;
    //private double _amplitude = 0.2;
    //private double _freqency = 0.1;
    //private int steps = 0;

    @Override
    public void run()
    {

    	jt.start();
    	
        //moveToInitialPosition();

        JointPosition initialPosition = new JointPosition(
                _theLbr.getCurrentJointPosition());
        DirectServo aSmartServoMotion = new DirectServo(initialPosition);

        //aSmartServoMotion.overrideJointAcceleration(1);
        //aSmartServoMotion.setJointJerk(1);
        
        double exec_time = 100e-3;
        aSmartServoMotion.setMinimumTrajectoryExecutionTime(exec_time);
        aSmartServoMotion.setTimeoutAfterGoalReach(3000);
        aSmartServoMotion.setJointVelocityRel(0.2);
        aSmartServoMotion.setJointAccelerationRel(0.2);
        //aSmartServoMotion.setSpeedTimeoutAfterGoalReach(80e-3);
        
        System.out.println("Starting SmartServo in Position Mode");
        _theLbr.moveAsync(aSmartServoMotion);

        // Fetch the Runtime of the Motion part
        theSmartServoRuntime = aSmartServoMotion.getRuntime();
        
        
        //theSmartServoRuntime.activateVelocityPlanning(true);
        
        // create an JointPosition Instance, to play with
        JointPosition destination = new JointPosition(
                _theLbr.getJointCount());
        JointPosition target_vel = new JointPosition(
                _theLbr.getJointCount());
        System.out.println("start loop");
        // For Roundtrip time measurement...
        //StatisticTimer timing = new StatisticTimer();
        try
        {
   
        	double kp[] = new double[7]; //0.005;
        	double kd[] = new double[7];
        	double amp[] = new double[7];
 	   	   int joint_id = 6;
        	kp[6] = 0.1;//0.04;
        	kd[6] = 0.0;
        	amp[6] = 2; //5
        	kp[5] = 0.04;
        	kd[5] = 0;
        	amp[5] = 2; //2
        	kp[4] = 0.06;//0.05;
        	kd[4] = 0;
        	amp[4] = 2;
        	kp[3] = 0.07;//0.003;
        	kd[3] = 0;
        	amp[3] = 2;
        	kp[2] = 0.05;//0.02;
        	kd[2] = 0;
        	amp[2] = 2;
        	kp[1] = 0.02;//0.003;
        	kd[1] = 0;
        	amp[1] = 2;
        	kp[0] = 0.06;//0.04;
        	kd[0] = 0;
        	amp[0] = 2;
        	int step_ctr=0;
        	System.out.println("starting servoing");
        	long t_last =System.nanoTime();
        	long t_here = t_last;

            JointPosition curMsrJntPose = theSmartServoRuntime
                     .getAxisQMsrOnController();
            StatisticTimer timing = new StatisticTimer();
            JointPosition prevMsrJntPose = new JointPosition(curMsrJntPose);
            JointPosition prevErr = new JointPosition(curMsrJntPose);

            boolean isFirstStep = true;
        	while(true) {

        	   OneTimeStep aStep = timing.newTimeStep();   
               //System.out.println("asking for joints");
        	   theSmartServoRuntime.updateWithRealtimeSystem();
        	   curMsrJntPose = theSmartServoRuntime
                       .getAxisQMsrOnController();
        	   
        	  
               target_vel = jt.getJP(curMsrJntPose);
              
        	  // theSmartServoRuntime.updateWithRealtimeSystem();
              // curMsrJntPose = theSmartServoRuntime
              //         .getAxisQMsrOnController();
               

               t_here = System.nanoTime();
               double dt = (t_here-t_last)/1e9;
               t_last = t_here;
               //double t = t_here/1e9;
               //if time step is too large top
               if(dt > 0.2) dt = 0;
               
               JointPosition curr_vel = new JointPosition(target_vel);
               
               JointPosition e = new JointPosition(target_vel);
               
               //JointPosition u = new JointPosition(target_vel);
               for (int i=0; i<_theLbr.getJointCount(); ++i) {
               		curr_vel.set(i, (curMsrJntPose.get(i)-prevMsrJntPose.get(i))/dt);
               		prevMsrJntPose.set(i,curMsrJntPose.get(i));
               		e.set(i,target_vel.get(i)-curr_vel.get(i));
               		if(isFirstStep) {
            	    	prevErr.set(i,e.get(i));
               		}
               		destination.set(i, kp[i]*e.get(i) + curMsrJntPose.get(i) + amp[i]*dt*target_vel.get(i));
               		prevErr.set(i,e.get(i));
               		
               		if(destination.get(i)-curMsrJntPose.get(i) < Math.toRadians(-4)) {
               			destination.set(i,curMsrJntPose.get(i)+Math.toRadians(-4));
               		}
               		if(destination.get(i)-curMsrJntPose.get(i) > Math.toRadians(4)) {
               			destination.set(i,curMsrJntPose.get(i)+Math.toRadians(4));
               		}
               	   /*if(Math.abs(destination.get(i)-curMsrJntPose.get(i)) < Math.toRadians(0.1)) {
               			destination.set(i,curMsrJntPose.get(i));
               	   }*/

               }
               isFirstStep=false;
               
               //aSmartServoMotion.setMinimumTrajectoryExecutionTime(dt/1000.);
              // for (int i=0; i<_theLbr.getJointCount(); ++i) {
            	   //calculate q_k+1 HERE
            	   
            	   //double jerk = 2*(target_vel.get(i) - curr_vel.get(i))/(exec_time*exec_time);
            	   //destination.set(i, curMsrJntPose.get(i)+exec_time*target_vel.get(i)+jerk/6*Math.pow(exec_time, 3));
            	//   destination.set(i, curMsrJntPose.get(i)+exec_time*target_vel.get(i));
            	                 		

               //}
               //System.out.println("sent to jp "+destination.get(6));
               
               //aSmartServoMotion.setMinimumTrajectoryExecutionTime(1.5*dt);
               theSmartServoRuntime.setDestination(destination);
               
               
               if(step_ctr%100 ==0) {
            	//   step_ctr=0;
            	   //for (int i=5; i<_theLbr.getJointCount(); ++i) {
         
            		   System.out.println("sent to jp "+joint_id+": "+destination.get(joint_id)+" dt is "+dt+"target vel is "+target_vel.get(joint_id)+" curr vel "+curr_vel.get(joint_id)+" curr pos "+curMsrJntPose.get(joint_id));
            	   //}
                   getLogger().info("Statistic Timing of Overall Loop " + timing);

               }
               step_ctr++;
               
               //theSmartServoRuntime.setDestination(curMsrJntPose,target_vel);
               
//               ThreadUtil.milliSleep(10);

               aStep.end(); 

            
               
               //js.setJS(curMsrJntPose);
      
        	}
        }
        catch (Exception e)
        {
            System.out.println(e);
            e.printStackTrace();
        }
        ThreadUtil.milliSleep(1000);
        // Stop the motion
        theSmartServoRuntime.stopMotion();
        try {
			jt.join();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
       
       
    }
    
 


	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		SmartServoControl app = new SmartServoControl();
		app.runApplication();
	}
}
