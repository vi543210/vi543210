#include <ros/ros.h>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <string> 
#include <vector>

using namespace std;

const double WheelDistance = 0.46;		//m
const double WheelRadius = 0.099;		//m
//const double GearRatio = 1.0;
const double DriverSampleTime = 0.066;		//s
const unsigned long fourthmultiple = pow(2, 24);
const unsigned int thirdmultiple = pow(2, 16);
const unsigned long secondmultiple = pow(2, 8);


//serial parameter
/////////////////////////
serial::Serial serL;
std::string Lport;
int baudrate;

bool writing = false;
bool speedset_flag = false;
bool initial_flag =false;

//odometry
//double Odometry[3] = {0.0};	//x,y,theta (m,m,rad) -pi~pi

//double deltatheta;
//double deltas ;


int counter = 0;
int init_counter = 0;
unsigned int checksum;
//int debug_flag=0;


//Motor Cmd
std_msgs::UInt8MultiArray lmotorcmd;

void initial();
void driverinitial();
void write_serial(const geometry_msgs::Twist::ConstPtr& speed);
void initialCB(const std_msgs::Bool::ConstPtr& msgs);


int main (int argc, char** argv)
{
    
	ros::init(argc, argv, "serial_driver_node");
	ros::NodeHandle nh;

	ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 5, write_serial);
	ros::Subscriber odom_initial_sub = nh.subscribe("odom_initial_flag", 1, initialCB);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 5);
	ros::Publisher odom_initial_pub = nh.advertise<nav_msgs::Odometry>("odom_initial", 5);
	tf::TransformBroadcaster odom_broadcaster;
	
	initial();
	//connect to serial
	try{
		//Left port
		serL.setPort(Lport);
		serL.setBaudrate(baudrate);
		serial::Timeout Lto = serial::Timeout::simpleTimeout(0);
		serL.setTimeout(Lto);
		serL.open();	
	}
	catch (serial::IOException& e){
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}
	//open or not
	if(serL.isOpen()){
		//serL.setBreak(true);
		ROS_INFO_STREAM("Serial Leftwheel Port initialized");
	}
	else{
		return -1;
	}
	ros::Rate loop_rate(10);
	
	//driverinitial();
		
	std_msgs::UInt8MultiArray array;
	std_msgs::UInt8MultiArray arrayR;
	int count[2] = {0, 0};
	
	//FirstEncoder
	vector<unsigned char> flencoder(4);
	vector<unsigned char> frencoder(4);
	long int fencoderPulse[2] = {0, 0}; 
	vector<bool> initial(2);  //initial[0] := left
	initial[0] = true;
	initial[1] = true;

	//CurrentEncoder
	vector<unsigned char> lencoder(4);
	vector<unsigned char> rencoder(4);
	long int encoderPulse[2] = {0, 0}; 
	
	//odometry
	double Odometry[3] = {0.0};	//x,y,theta (m,m,rad) -pi~pi
	double Odometry_initial[3] = {0.0};	//x,y,theta (m,m,rad) -pi~pi
	
	ros::Time current_time;
	while(ros::ok()){
		//Receive data have to delay!! very important!!
		//ros::Duration(0.05).sleep();
		while(1){
			
			if(serL.available() >= 1){
				array.data.clear();
				serL.read(array.data,1);
				if(array.data[0]==0x4A)
				{
					//std::cout<<"==============="<<endl;	
					//std::cout<<"gogo"<<endl;					
					//std::cout<<"==============="<<endl;	
					//std::cout<<"**********************************************************"<<counter<<endl;			
					break;			
										
				}
							
			}

		}

		while(1){
			
			if(serL.available() >= 25){
				array.data.clear();
				//read 25 unsigned char
				serL.read(array.data,25);
				checksum=0x4A^array.data[0]^array.data[1]^array.data[2]^array.data[3]^array.data[4]^array.data[5]^array.data[6]^array.data[7]^array.data[8]^array.data[9]^array.data[10]^array.data[11]^array.data[12]^array.data[13]^array.data[14]^array.data[15]^array.data[16]^array.data[17]^array.data[18]^array.data[19]^array.data[20]^array.data[21]^array.data[22]^array.data[23];
				if((array.data[23]==0x44)&&(array.data[24]==checksum)){
				//清調input buffer
				serL.flushInput();
				for(int i = 2; i < 6; i++){
					rencoder[i-2] = array.data[i];
				}
				for(int i = 6; i < 10; i++){
					lencoder[i-6] = array.data[i];
				}
				
				//std::cout<<"RightEncoderInfo=" <<  (int)(unsigned int)array.data[2] <<" ;"<<(int)(unsigned int)array.data[3] <<" ;" <<(int)(unsigned int)array.data[4] <<" ;" <<(int)(unsigned int)array.data[5] << " ;"  <<  "\n";
				//std::cout<<"LeftEncoderInfo=" << (int)(unsigned int)array.data[6] <<" ;" <<(int)(unsigned int)array.data[7] <<" ;" <<(int)(unsigned int)array.data[8] <<" ;" << (int)(unsigned int)array.data[9] <<" ;"<<"\n";
				if(initial[0] == true){
					for(int i = 2; i < 6; i++){
						frencoder[i-2] = rencoder[i];
					}
				}
				if(initial[1] == true){
					for(int i = 6; i < 10; i++){
						flencoder[i-6] = lencoder[i];
					}
				}
				break;	
				}
				
				
			}else{
				count[0]++;
				if(count[0] >= 3){
					//sleep(5);
					count[0] = 0;
					break;
				}
			}
		
		}
		
#if 0		
		//Display initial encoder
		cout << "InitialEncoderInfo\nLeft :";
		for(int i = 0; i < 4; i++){
			cout << (int)(unsigned int)flencoder[i] << ", ";
		}
		cout << "\nRight : ";
		for(int i = 0; i < 4; i++){
			cout << (int)(unsigned int)frencoder[i] << ", ";
		}
		cout << "\n\n";
#endif
		
		//Compute Odometry
		/////////////////////////////////////////
		current_time = ros::Time::now();
		//cout << "Multiple:" << fourthmultiple << " ;" << thirdmultiple << " ;" << secondmultiple << "\n";
		
		//left
		if ((~lencoder[3]+1) < 0){
			encoderPulse[0] = lencoder[3] * fourthmultiple + lencoder[2] * thirdmultiple +lencoder[1] * secondmultiple + lencoder[0] - 4294967296;
		}else{
			encoderPulse[0] = lencoder[3] * fourthmultiple + lencoder[2] * thirdmultiple +lencoder[1] * secondmultiple + lencoder[0];
		}
		//right
		if ((~rencoder[3]+1) < 0){
			encoderPulse[1] = rencoder[3] * fourthmultiple + rencoder[2] * thirdmultiple +rencoder[1] * secondmultiple + rencoder[0] - 4294967296;
		}else{
			encoderPulse[1] = rencoder[3] * fourthmultiple + rencoder[2] * thirdmultiple +rencoder[1] * secondmultiple + rencoder[0];
		}
#if 1
		//initialvalue
		if(initial[0] == true){
			fencoderPulse[0] = encoderPulse[0];
			fencoderPulse[1] = encoderPulse[1];
			cout << "hhello\n";
			init_counter++;
			if(init_counter==3){
				initial[0] = false;
				initial[1] = false;
			}
		}	
		double WPulse[2]={0,0};
		WPulse[0]=(encoderPulse[0] - fencoderPulse[0])*0.001;
		WPulse[1]=(encoderPulse[1] - fencoderPulse[1])*0.001;
		
		fencoderPulse[0] = encoderPulse[0];
		fencoderPulse[1] = encoderPulse[1];
		//std::cout<<"************************************"<<endl;		
		//cout << "WPulse : " << WPulse[0] << ", WPulse : " << WPulse[1] << "\n\n";
#endif
		//std::cout<<"************************************"<<endl;		
		//cout << "LencoderPulse : " << encoderPulse[0] << ", RencoderPulse : " << encoderPulse[1] << "\n\n";
		//std::cout<<"************************************"<<endl;
		//Odometry
		//cout << "Odometry : \n";
		
		double WV[2] = { 0.0, 0.0 };
		double old_WV[2] = { 0.0, 0.0 };
		
		//WV[0] = ((double)encoderPulse[0] / EncoderPlus) * (2.0 * M_PI * WheelRadius) / GearRatio / DriverSampleTime;
		//WV[1] = ((double)encoderPulse[1] / EncoderPlus) * (2.0 * M_PI * WheelRadius) / GearRatio / DriverSampleTime;
		
		//1.3是tune的值
		WV[0] = (double)WPulse[0] / DriverSampleTime *1.3;
		WV[1] = (double)WPulse[1] / DriverSampleTime *1.3;
		
		//new_sun
		printf("%ld \n",encoderPulse[0]);			
		
		if((WV[0]>10)||(WV[1]>=10)||(WV[0]<=-10)||(WV[1]<=-10)){
			WV[0]=old_WV[0];
			WV[1]=old_WV[1];
			//printf("fuck  %d   %d\n",WV[0]);
			printf("r1 : %x \n",array.data[2]);
			printf("r2 : %x \n",array.data[3]);
			printf("r3 : %x \n",array.data[4]);
			printf("r4 : %x \n",array.data[5]);
			printf("l1 : %x \n",array.data[6]);
			printf("l2 : %x \n",array.data[7]);
			printf("l3 : %x \n",array.data[8]);
			printf("l4 : %x \n",array.data[9]);
			printf("encoderr  %ld \n",encoderPulse[0]);
			printf("encoderl  %ld \n",encoderPulse[1]);
			//sleep(5000);
						
		}
		else{
			old_WV[0]=WV[0];
			old_WV[1]=WV[1];
		}
		



		//initial_odom
		if(initial_flag){
			Odometry_initial[0] = 0.0;
			Odometry_initial[1] = 0.0;
			Odometry_initial[2] = 0.0;
			initial_flag=false;
			ROS_ERROR("ENTER");

		}


		double LinearVelocity = (WV[1] + WV[0]) / 2.0 ;
		double AngularVelocity = (WV[1] - WV[0]) / WheelDistance;
		
		double deltatheta = (WV[1] - WV[0]) * DriverSampleTime / WheelDistance;//角度的大小
		double deltas = (WV[1] + WV[0]) * DriverSampleTime / 2.0;//linear走得距離
		
		double thetak = Odometry[2];	//theta of last time
		double thetak_initial = Odometry_initial[2];
		Odometry[2] = (Odometry[2] + deltatheta) *1;
		Odometry_initial[2] = (Odometry_initial[2] + deltatheta) *1;
		Odometry[0] = (Odometry[0] + deltas * cos((Odometry[2] + thetak) / 2.0)) * 1;
		Odometry_initial[0] = (Odometry_initial[0] + deltas * cos((Odometry_initial[2] + thetak_initial) / 2.0)) * 1;
		Odometry[1] = Odometry[1] + deltas * sin((Odometry[2] + thetak) / 2.0);
		Odometry_initial[1] = Odometry_initial[1] + deltas * sin((Odometry_initial[2] + thetak_initial) / 2.0);


		if (Odometry[2] > M_PI)
			Odometry[2] -= (2.0 * M_PI);
		else if (Odometry[2] < -M_PI)
			Odometry[2] += (2.0 * M_PI);
		if (Odometry_initial[2] > M_PI)
			Odometry_initial[2] -= (2.0 * M_PI);
		else if (Odometry_initial [2]< -M_PI)
			Odometry_initial[2] += (2.0 * M_PI);


		ROS_ERROR("x: %.2f y: %.2f  theta:%.2f",Odometry_initial[0],Odometry_initial[1],Odometry_initial[2]);
		//ROS_INFO_STREAM("odom x: " << Odometry[0] << " odom y: " << Odometry[1] << " odom theta: " << Odometry[2] * 180.0 / M_PI);
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Odometry[2]);
		geometry_msgs::Quaternion odom_quat_initial = tf::createQuaternionMsgFromYaw(Odometry_initial[2]);
			
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";
		odom_trans.transform.translation.x = Odometry[0];
		odom_trans.transform.translation.y = Odometry[1];
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
			

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = Odometry[0];
		odom.pose.pose.position.y = Odometry[1];
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_footprint";
		odom.twist.twist.linear.x = LinearVelocity;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.angular.z = AngularVelocity;

		//publish the message
		
		odom_pub.publish(odom);

		//odom_initial
		nav_msgs::Odometry odom_initial;
		odom_initial.header.stamp = current_time;
		odom_initial.header.frame_id = " ";

		//set the position
		odom_initial.pose.pose.position.x = Odometry_initial[0];
		odom_initial.pose.pose.position.y = Odometry_initial[1];
		odom_initial.pose.pose.position.z = 0.0;
		odom_initial.pose.pose.orientation = odom_quat_initial;

		//set the velocity
		odom_initial.child_frame_id = " ";
		odom_initial.twist.twist.linear.x = LinearVelocity;
		odom_initial.twist.twist.linear.y = 0.0;
		odom_initial.twist.twist.angular.z = AngularVelocity;

		//publish the message
		
		odom_initial_pub.publish(odom_initial);
		
	
		loop_rate.sleep();
		
		ros::spinOnce();

	}
	serL.close();
	//serR.close();
}	


void initial(){
	ros::NodeHandle para_node("~");
	if(!para_node.getParam("Lport", Lport))
    		Lport = "/dev/ttyUSB0";
	if(!para_node.getParam("baudrate", baudrate))
    		baudrate = 115200;
}

void driverinitial(){

	std_msgs::UInt8MultiArray array;
	
}
void initialCB(const std_msgs::Bool::ConstPtr& msgs){
	if(msgs->data==true)
		initial_flag=true;
	else
		initial_flag=false;
}
void write_serial(const geometry_msgs::Twist::ConstPtr& speed)
{
		
	unsigned int MotorCmd[9] = { 0x4A, 0x01, 0x40, 0x00, 0x00, 0x00 ,0x00,0x44,0x00};	
	//char r_hex[4];
	//char l_hex[4];

	double vr = (speed->linear.x * 2.0 + speed->angular.z * WheelDistance) / 2.0;
	double vl = (speed->linear.x * 2.0 - speed->angular.z * WheelDistance) / 2.0;
	int leftmotorvalue = (int)(round(vl / (2.0 * M_PI *  WheelRadius) * 60.0));
	int rightmotorvalue = (int)(round(vr / (2.0 * M_PI *  WheelRadius) * 60.0));
	if (leftmotorvalue > 250)
		leftmotorvalue = 250;
	else if (leftmotorvalue < -250)
		leftmotorvalue = -250;

	if (rightmotorvalue > 250)
		rightmotorvalue = 250;
	else if (rightmotorvalue < -250)
		rightmotorvalue = -250;
	//int motorLcmd = (int)(round(leftmotorvalue * 16384.0 / 6000.0));
	//int motorRcmd = (int)(round(rightmotorvalue * 16384.0 / 6000.0));
	int motorLcmd = (int)(round(leftmotorvalue * 25));
	int motorRcmd = (int)(round(rightmotorvalue * 25));
	
	
	//std::cout<<"RMotorRPM = "<< rightmotorvalue <<" ;LMotorRPM = "<< leftmotorvalue << " ;" << " ;motorRcmd = "<< motorRcmd << " ;motorLcmd = "<< motorLcmd << "\n";
	//std::cout<<"RMotorRPM = "<< rightmotorvalue <<" ;LMotorRPM = "<< leftmotorvalue <<"\n";
	MotorCmd[3] = motorRcmd&0XFF;	//low 8
  	MotorCmd[4] = motorRcmd>>8;	//high 8
	MotorCmd[5] = motorLcmd&0XFF;	//low 8
  	MotorCmd[6] = motorLcmd>>8;	//high 8
	MotorCmd[8]=MotorCmd[0]^MotorCmd[1]^MotorCmd[2]^MotorCmd[3]^MotorCmd[4]^MotorCmd[5]^MotorCmd[6]^MotorCmd[7];
	lmotorcmd.data.clear();
	lmotorcmd.data.push_back(MotorCmd[0]);
	lmotorcmd.data.push_back(MotorCmd[1]);
	lmotorcmd.data.push_back(MotorCmd[2]);
	lmotorcmd.data.push_back(MotorCmd[3]);
	lmotorcmd.data.push_back(MotorCmd[4]);
	lmotorcmd.data.push_back(MotorCmd[5]);
	lmotorcmd.data.push_back(MotorCmd[6]);
	lmotorcmd.data.push_back(MotorCmd[7]);
	lmotorcmd.data.push_back(MotorCmd[8]);
	serL.write(lmotorcmd.data);


}

