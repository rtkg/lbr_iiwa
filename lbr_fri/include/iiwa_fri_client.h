#ifndef _IIWA_FRI_CLIENT_H
#define _IIWA_FRI_CLIENT_H

#include "friLBRClient.h"
#include <sensor_msgs/JointState.h>
#include <string>
#include <Server.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>

using namespace KUKA::FRI;

/**
 * \brief Test client that can overlay interpolator joint positions with sine waves.
 */
class IIWAFRIClient : public LBRClient
{
   
public:
   
    IIWAFRIClient () { 
	joint_names.push_back("lbr_iiwa_joint_1");
	joint_names.push_back("lbr_iiwa_joint_2");
	joint_names.push_back("lbr_iiwa_joint_3");
	joint_names.push_back("lbr_iiwa_joint_4");
	joint_names.push_back("lbr_iiwa_joint_5");
	joint_names.push_back("lbr_iiwa_joint_6");
	joint_names.push_back("lbr_iiwa_joint_7");
	for(int i=0; i<7; ++i) {
	    joint_targets[i] = 0;
	    joint_increment[i] = 0;
	}
	//per joint
	max_incr[0] = 0.0058;
	min_incr[0] = 0.0001;
	max_incr[1] = 0.0058;
	min_incr[1] = 0.001;
	max_incr[2] = 0.0069;
	min_incr[2] = 0.0001;
	max_incr[3] = 0.0052;
	min_incr[3] = 0.0002;
	max_incr[4] = 0.009;
	min_incr[4] = 0.0001;
	max_incr[5] = 0.009;
	min_incr[5] = 0.0001;
	max_incr[6] = 0.009;
	min_incr[6] = 0.0001;
    }; 
    ~IIWAFRIClient () { };   
   
    /**
    * \brief Callback for FRI state changes.
    * 
    * @param oldState
    * @param newState
    */
   virtual void onStateChange(ESessionState oldState, ESessionState newState);
   
   /**
    * \brief Callback for the FRI state 'Commanding Active'.
    */
   virtual void command();
   
   /**
    * \brief Callback for the FRI state 'Monitoring Active'.
    */
   virtual void monitor();

   void getJointMsg(sensor_msgs::JointState &msg);
   void getJointsRaw(double (&pos)[7], double (&vel)[7], double (&eff)[7]);
   
   void setJointTargets(const double (&com)[7]);
   
   ros::Time getTime(){
       //ros::Time t (robotState().getTimestampSec(), robotState().getTimestampNanoSec());
       //return t;
       return ros::Time::now();
   };
   ros::Duration getPeriod() {
	//return ros::Duration(robotState().getSampleTime());
       return ros::Duration(period);
   }
private:
   double joint_pos[7];
   double joint_increment[7];
   double joint_pos_interp[7];
   double max_incr[7];
   double min_incr[7];
   double joint_torques[7];
   double joint_targets[7];
   std::vector<std::string> joint_names;
   double period;
   double last_time;
   double getDoubleTime()
   {
       struct timeval time;
       gettimeofday(&time,NULL);
       return time.tv_sec + time.tv_usec * 1e-6;
   }
};

class IIWAFRIClientNative
{
   
public:
   
    IIWAFRIClientNative () { 
	joint_names.push_back("lbr_iiwa_joint_1");
	joint_names.push_back("lbr_iiwa_joint_2");
	joint_names.push_back("lbr_iiwa_joint_3");
	joint_names.push_back("lbr_iiwa_joint_4");
	joint_names.push_back("lbr_iiwa_joint_5");
	joint_names.push_back("lbr_iiwa_joint_6");
	joint_names.push_back("lbr_iiwa_joint_7");
	for(int i=0; i<7; ++i) joint_targets[i] = 0;
	SIZE = 7;
	send_established = false;
	recv_established = false;
	last_read_time = 0;
	period = 0;
    }; 
    ~IIWAFRIClientNative () { 
	joinThreads();
	if(joints_recv != NULL) delete joints_recv;
	if(command_send != NULL) delete command_send; 
    };   
  
   virtual void startThreads() {
        recv_quit = false;
	send_quit = false;
	doStep = false;
	send_thread = boost::thread(boost::bind(&IIWAFRIClientNative::commandThread,this));
	recv_thread = boost::thread(boost::bind(&IIWAFRIClientNative::monitorThread,this));
   } 
   virtual void joinThreads() {
	js_m.lock();
	recv_quit = true;
	js_m.unlock();

	jt_m.lock();
	send_quit=true;
	jt_m.lock();

	send_thread.join();
	recv_thread.join();
   }

   //signals
   void step() {
       //wakes up everyone waiting for a socket step
       boost::mutex::scoped_lock lock(socket_m);
       doStep=true;
       socket_cond_.notify_all();
   }

   bool waitForSession() {
       boost::mutex::scoped_lock lock(session_m);
       while(!(send_established && recv_established)) {
	   session_cond_.wait(lock);
       }
   
       return true; 
   }
   /**
    * thread that sends joint commands 
    */
   virtual void commandThread();
   
   /**
    * thread that reads joint values 
    */
   virtual void monitorThread();

   void getJointMsg(sensor_msgs::JointState &msg);
   void getJointsRaw(double (&pos)[7], double (&vel)[7], double (&eff)[7]);
   void setJointTargets(double (&com)[7]);
   
   ros::Time getTime(){
       t_m.lock();
       ros::Time t (last_read_time);
       t_m.unlock();
       return t;
   };
   ros::Duration getPeriod() {
       t_m.lock();
       ros::Duration d (period); 
       t_m.unlock();
       return d;
   }
private:
   double joint_pos[7];
   double joint_torques[7];
   double joint_targets[7];
   std::vector<std::string> joint_names;
   double period, last_read_time;
   Server *joints_recv, *command_send;
   boost::mutex js_m, jt_m, t_m, session_m, socket_m;
   boost::thread send_thread, recv_thread;
   boost::condition_variable session_cond_;
   boost::condition_variable socket_cond_;

   bool recv_quit, send_quit, doStep;
   bool send_established, recv_established;
   int SIZE;

   double getDoubleTime()
   {
       struct timeval time;
       gettimeofday(&time,NULL);
       return time.tv_sec + time.tv_usec * 1e-6;
   }

};

#endif // _IIWA_FRI_CLIENT_H
