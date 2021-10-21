#include "visualizer.h"


#define NUM_ROBOTS 2



#define PIXEL_CONVERSION 1.0

Robot::Robot(int robot_id)
	{
			id = robot_id;
			comm_thread_running = false;

			sleep_time = 200000/3;
			linear_velocity = 1.0 ;
			dist_travelled = 0.0 ;
			path_length = 0.0 ;
			prev_target_x = -100;	
			prev_target_y = -100;
			turn = false; 

			slow_down = 0 ;

	}

Robot::~Robot()
	{
			if(comm_thread_running)
			{
				comm_thread_running = false;
				comm_thread_send->join();
			}
	}

		

void Robot::start_comm()
	{
			comm_thread_running = true;
			comm_thread_send = new boost::thread(boost::bind(&Robot::comm_send,this));
	}
		

void Robot::calcPathDistance()
	{
			init_x = x;
			init_y = y;
			path_length = fabs(init_x - target_x) + fabs(init_y - target_y);
			acc_point = 0.5 ;
			dec_point = path_length - 0.5;
	}

void Robot::calcPathTravelledLinearVelocity()
	{
			dist_travelled = fabs(init_x - x) + fabs(init_y -y) ;
			if(dist_travelled >= dec_point)
			{

				linear_velocity = 0.4 ;
				return ;
			}
			linear_velocity = sqrt(2*1*dist_travelled) ;
			if(linear_velocity < 0.5 )
				linear_velocity  = 0.2;
			if(linear_velocity > 1.0)
				linear_velocity = 1.0;
	}

void Robot::comm_send()
	{
			while(comm_thread_running)
			{
				if(target_theta < 0.0)
					target_theta = 6.28 + target_theta;

				if(fabs(target_theta - theta) < 0.06 || (fabs(6.28 - target_theta -theta) < 0.06)){
					theta = target_theta;
					turn = false ;
				}
				else
				{
					float alt_theta = 6.28 - target_theta;
					bool use_alt = false;
					if(fabs(target_theta - theta) <= fabs(alt_theta - theta))
					{
						use_alt = false;
					}
					else if(fabs(target_theta - theta) > fabs(alt_theta - theta))
					{
						use_alt = true;
					}

					if(use_alt)
					{
						theta += sign(alt_theta - theta) * 0.05;
					}
					else
					{
						theta += sign(target_theta - theta) * 0.05;
					}
					turn = true ;

				}
				if(target_x != prev_target_x || target_y != prev_target_y)
				{
					prev_target_x = target_x ;
					prev_target_y = target_y ;
					calcPathDistance() ;
				}
				if(fabs(target_theta - theta) < 0.05)
				{
					calcPathTravelledLinearVelocity();
					linear_velocity = 1.0;
					if(fabs(target_x - x) < 0.05)
						x = target_x;
					else
					{
						if(target_x > x)
							x = x + 0.1;
						else if(target_x < x)
							x= x - 0.1;

					}
					if(fabs(target_y - y) < 0.05)
						y = target_y;
					else
					{
						if(target_y > y)
							y = y + 0.1;
						else if(target_y < y)
							y = y - 0.1;
					}
				}


				if(slow_down == 1)	
					sleep_time = (0.1/0.2) * 1000000 ;
				if(turn)
					sleep_time = (0.1/2.5) * 1000000 ;
				usleep(sleep_time);


			}
	}

float Robot::sign(float x)
	{
			if(x >= 0.0)
				return 1;
			else
				return -1;
	}

void Robot::set_x_y_theta(float curr_x, float curr_y, float curr_theta)
	{
			x = curr_x;
			y = curr_y;
			theta = curr_theta;
	}
vector<double> Robot::get_x_y_theta()
	{
			std::vector<double> v;
			v.push_back(x);
			v.push_back(y);
			v.push_back(theta);
			return v;
	}

void Robot::set_target_x_y_theta(float curr_x, float curr_y, float curr_theta)
	{
			target_x = curr_x;
			target_y = curr_y;
			target_theta = curr_theta;
	}
		






Planner::Planner(int robot_id,std::shared_ptr<Robot> rbot)
	{
			robot =rbot;
			id = robot_id;
			comm_thread_running = false;
			printf("Planner %d created\n", id);			
			prev_target_x = -100;	
			prev_target_y = -100;
			read_path(id+1);

	}

Planner::~Planner()
	{
			if(comm_thread_running)
			{
				comm_thread_running = false;
				comm_thread_send->join();
				comm_thread_recv->join();
			}
	}

		
void Planner::start_comm()
	{
			comm_thread_running = true;
			comm_thread_send = new boost::thread(boost::bind(&Planner::comm_send,this));
			comm_thread_recv = new boost::thread(boost::bind(&Planner::comm_recv,this));
	}
		


void Planner::comm_send()
	{
			int counter =1;
			while(comm_thread_running)
			{
				double path_length = fabs(x - current_full_path[counter][0]) + fabs(y-current_full_path[counter][1]);
				if((path_length<0.02)&&(int(current_full_path[counter][2]*100) == int(theta*100))){
					counter =counter+1;
				}
				robot.get()->set_target_x_y_theta(current_full_path[counter][0],current_full_path[counter][1],current_full_path[counter][2]);
				if(counter == (current_full_path.size()-1)){
					counter =1;
				}
			}
	}

void Planner::comm_recv()
	{
			while(comm_thread_running)
			{
				std::vector<double> v =robot.get()->get_x_y_theta();
				x=v[0];
				y=v[1];
				theta=v[2];
				usleep(200000/3);
			}
	}
void Planner::read_path(int robot_ID)
    {
            FILE *fp = NULL;
            std::string filepath = "../config/" + std::to_string(robot_ID) + ".txt";
            fp = fopen(filepath.c_str(),"r");
            if (fp == NULL)
            {	std::cout << "error:FILE PATH FOR BIN LOCATION NOT CORRECT " << std::endl;
                exit(1);
            }
            for(int i = 0; i < current_full_path.size(); i++)
			{		
				current_full_path[i].clear();
				current_full_path[i].resize(0);
			}
            while(1)	
            {
                char name[10];
                double x,y,theta;
                int r = fscanf(fp,"%lf,%lf,%lf,\n",&x,&y,&theta);
                if (r == EOF)
                    break;
				std::vector<double> v;
				v.push_back(x);
				v.push_back(y);
				v.push_back(theta);
                if(v.size() == 3){
					current_full_path.push_back(v);
				}
                v.clear();
            }
    }
		


std::vector<float> starting_pos;

void get_robot_starting_pos()
{
	std::ifstream file;
	std::string word, t, q, filename; 
	filename = "../config/initial_poses.txt"; 
	file.open(filename.c_str()); 
	char charsToRemove[] = " ,";
	while (std::getline(file, word, ',')) 
	{ 

		for ( unsigned int i = 0; i < strlen(charsToRemove); ++i ) 
		{
			word.erase(remove(word.begin(), word.end(), charsToRemove[i]), word.end() );
		}
		starting_pos.push_back(std::stof(word));
		printf("Pushed back %s %f\n", word.c_str(), starting_pos.back());
	}
}

void plotPointImg(Mat img, int x, int y, int degrees, Scalar color) {
	cv::Point center = Point(x,y);

	Scalar color_existing;
	color_existing.val[0] = img.data[y*img.cols*3 + x*3 + 0];
	color_existing.val[1] = img.data[y*img.cols*3 + x*3 + 1];
	color_existing.val[2] = img.data[y*img.cols*3 + x*3 + 2];

	Scalar white;
	white.val[0] = 255.0;
	white.val[1] = 255.0;
	white.val[2] = 255.0;

	Scalar purple ;
	purple.val[0] = 211.0;
	purple.val[1] = 0.0;
	purple.val[2] = 211.0;

	if((color_existing.val[0] != white.val[0]) && (color_existing.val[1] != white.val[1]) && (color_existing.val[2] != white.val[2]) && (color_existing.val[0] != purple.val[0]) && (color_existing.val[1] != purple.val[1]) && (color_existing.val[2] != purple.val[2]))
		circle(img, center, 30, color);
	RotatedRect rRect = RotatedRect(center, Size2f(6,4), degrees);
	Point2f vertices[4];
	rRect.points(vertices);
	for (int i = 0; i < 4; i++)
		line(img, vertices[i], vertices[(i+1)%4], color, 3);

	string s ="Visualiser";  
	char msg1[1024];
	sprintf(msg1,s.c_str());
	putText(img, msg1, cv::Point(img.size().width*0.2,img.size().height*0.92), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,0,0), 1, cv::LINE_AA);

}

//read all bin locations from text file 
void read_drop_point(std::vector<std::vector<double>> &drop_points)
{
	FILE *fp = NULL;
	fp = fopen("../config/bin_pos.txt","r");
	if (fp == NULL)
	{	std::cout << "error:FILE PATH FOR BIN LOCATION NOT CORRECT " << std::endl;
		exit(1);
	}
	while(1)	
	{
		char name[10];
		double x,y,theta;
		int r = fscanf(fp,"%s %lf %lf %lf\n",name,&x,&y,&theta);
		if (r == EOF)
			break;
		std::vector<double> vec;
		vec.push_back(x);
		vec.push_back(y);
		drop_points.push_back(vec);
		vec.clear();
	}
}
//read all pick location from text file
void read_pick_point(std::vector<std::vector<double>> &pick_points)
{
	FILE *fp = NULL ;
	fp = fopen("../config/pick_pos.txt","r");
	if(fp == NULL)
	{
		exit(1);
	}
	while(1)
	{
		char name[10];
		double x,y,theta;
		int r = fscanf(fp,"%s %lf %lf %lf\n",name,&x,&y,&theta);
		if (r== EOF)
			break ;
		std::vector<double> vec;
		vec.push_back(x);
		vec.push_back(y);
		pick_points.push_back(vec);
		vec.clear();
	}
}


int main()
{

	std::vector <std::vector <double>>  drop_points ;
	read_drop_point(drop_points);
	std::vector <std::vector <double>>  pick_points;
	read_pick_point(pick_points) ;
	RNG rng(12345);
	std::vector <std::vector <double>>  grid_points ;

	get_robot_starting_pos();
	std::vector<std::shared_ptr<Robot>> robots;// = std::make_shared<std::vector<Robot>>();
	std::vector<Planner *> planner;
	for(int i = 0; i < NUM_ROBOTS; i++)
	{
		std::shared_ptr<Robot> test;
		robots.push_back(test);
		printf("Created %d robot at %f %f %f\n", i+1, starting_pos[i*3], starting_pos[i*3 + 1], starting_pos[i*3 + 2]);
		robots[i] = std::make_shared<Robot>(i);
		robots[i]->set_x_y_theta(starting_pos[i*3], starting_pos[i*3 + 1], starting_pos[i*3 + 2]);
		robots[i]->set_target_x_y_theta(starting_pos[i*3], starting_pos[i*3 + 1], starting_pos[i*3 + 2]);
		robots[i]->color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
		robots[i]->start_comm();
	}
	for(int i = 0; i < NUM_ROBOTS; i++)
	{
		Planner *plan;
		planner.push_back(plan);
		printf("Created %d planner at %f %f %f\n", i+1, starting_pos[i*3], starting_pos[i*3 + 1], starting_pos[i*3 + 2]);
		planner[i] = new Planner(i,robots[i]);
		planner[i]->start_comm();
	}
	Mat img; // image object

	const string windowName = "Visualiser"; // window name   
	img = imread("../config/env.pgm", IMREAD_COLOR);
	float blow_out = 30;		;
	printf("original image %d %d\n", img.size().width, img.size().height);
	Size size((int)(img.size().width * blow_out), (int)(img.size().height * blow_out)); // the resize image size,e.g.800x800

	printf("%d---------------%d\n",(int)(img.size().width * blow_out),(int)(img.size().height * blow_out));
	printf("BLown out image %d %d\n", size.width, size.height);
	Mat image; // destinations image
	resize(img, image, size); // resize image
	threshold(image, image, 127, 255, THRESH_BINARY);

	for(int i = 0; i< drop_points.size() ;i ++)
	{
		int x =(drop_points[i][0] / PIXEL_CONVERSION ) * blow_out ;
		int y =((drop_points[i][1])/ PIXEL_CONVERSION ) * blow_out ;
		cv::Point center = Point(x,y);
		circle(image, center, 6, cv::Scalar(211,211,211),-1);
		std::string str = std::to_string(i);
	}
	for(int i = 0; i< pick_points.size() ;i ++)
	{
		int x =(pick_points[i][0] / PIXEL_CONVERSION ) * blow_out ;
		int y =((pick_points[i][1])/ PIXEL_CONVERSION ) * blow_out ;
		cv::Point center = Point(x,y);
		circle(image, center, 6, cv::Scalar(200,0,0),-1);
		std::string str = std::to_string(i);
	}



	Mat temp = image.clone();
	while(waitKey(30) != 33)
	{
		temp = image.clone();
		for(int i = 0; i <robots.size(); i++)
		{
			int x = (int)(((robots[i]->x ) / PIXEL_CONVERSION) * blow_out);
			int y = (int)(((robots[i]->y )/ PIXEL_CONVERSION) * blow_out);
			int degrees = (int)(robots[i]->theta * 180.0 / 3.14);
			plotPointImg(temp, x, y, degrees, robots[i]->color);
		}

		namedWindow(windowName, 1);
		imshow(windowName, temp);
	}
	for(int i = 0; i < robots.size(); i++)
		delete(planner[i]);
}
