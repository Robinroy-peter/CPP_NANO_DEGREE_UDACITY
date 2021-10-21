#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <boost/thread.hpp>
#include <future>
#include <bits/stdc++.h> 
#include <fstream>
#include <iostream>
#include<sstream>  
#include<memory>
#include <mutex>
#include <vector>
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

class Robot
{
        public :
                Robot(int robot_id);
				~Robot();
				void start_comm();
				void calcPathDistance();
				void calcPathTravelledLinearVelocity();
				void comm_send();
				float sign(float x);
				void set_x_y_theta(float curr_x, float curr_y, float curr_theta);
				vector<double> get_x_y_theta();
				void set_target_x_y_theta(float curr_x, float curr_y, float curr_theta);
				Scalar color;
				float x;
				float y;
				float theta;
				float target_x;
				float target_y;
				float target_theta;

		private :
				int slow_down ;
				int id;
				bool comm_thread_running;
				boost::thread *comm_thread_send;
				long int sleep_time;
				double path_length;
				double dist_travelled;
				double linear_velocity; 
				float init_x;
				float init_y ;
				float prev_target_x;
				float prev_target_y;
				float acc_point;
				float dec_point;
				bool turn ;

		
};
class Planner
{
        public :
                Planner(int robot_id,std::shared_ptr<Robot> rbot);
				~Planner();
				void start_comm();
				void comm_send();
				void comm_recv();
				void read_path(int robot_ID);

				float x;
				float y;
				float theta;
				float target_x;
				float target_y;
				float target_theta;
				int id;
				bool comm_thread_running;
				boost::thread *comm_thread_send, *comm_thread_recv ;


				float init_x;
				float init_y ;
				float prev_target_x;
				float prev_target_y;
				std::vector<std::vector<double> > current_full_path;
				std::shared_ptr<Robot> robot;

		private :
};
