#include<vector>
#include<stdint.h>
#include<cmath>
#include<cstdio>
#include<string>

using namespace std;

#define DIS_INCREMENT 0.1

#define ROWS 4
#define COLS 2
#define DIS_INCREMENT 0.1
#define DIR_INVALID 255


typedef struct 
{
	uint8_t row;
	uint8_t col;
} vertexIndex_t;

typedef struct
{
	double longitude;
	double latitude;
	float yaw;
}gpsMsg_t;

gpsMsg_t path_vertexes_[ROWS][COLS];
std::vector<gpsMsg_t> path_points_;
uint8_t traffic_dir = DIR_INVALID;

vertexIndex_t target_vertex_index_ ={0,0};
vertexIndex_t last_vertex_index_ = {-1,0};


float disBetween2Points(const gpsMsg_t &start, const gpsMsg_t &end ,bool orientation=false)
{
	float x = (end.longitude -start.longitude)*111000*cos(end.latitude*M_PI/180.0);
	float y = (end.latitude - start.latitude ) *111000;
	
	float dis = sqrt(x * x + y * y);
	float yaw = atan2(x,y) *180.0/M_PI;
	if(yaw <0)
		yaw += 360.0;
	if(!orientation)
		return dis;
	
	if(fabs(yaw- start.yaw)>90.0)
		dis *= -1;
	return dis;
}


bool generateOriginPathPoints(std::vector<gpsMsg_t>& path_points)
{
	gpsMsg_t start_point ;
	start_point.longitude = 118.8101;
	start_point.latitude = 31.8864;
	
	gpsMsg_t end_point = path_vertexes_[0][0];
	
	int points_cnt = disBetween2Points(start_point,end_point)/DIS_INCREMENT;
	printf("points_cnt:%d",points_cnt);
	
	if(points_cnt < 0)
	{
		printf("generateOriginPathPoints failed");
		return false;
	}
	
	double longitude_increment = (end_point.longitude - start_point.longitude)/points_cnt;
	double latitude_increment  = (end_point.latitude - start_point.latitude)/points_cnt;
	for(size_t i=0; i<points_cnt; i++)
	{
		gpsMsg_t point;
		point.longitude = start_point.longitude + longitude_increment *i;
		point.latitude = start_point.latitude + latitude_increment *i;
		path_points.push_back(point);
	}
	return true;
}

uint8_t generateCurrentDir(const vertexIndex_t& target_index, const vertexIndex_t& last_index)
{
	uint8_t row_diff = target_index.row-last_index.row;
	uint8_t col_diff = target_index.col-last_index.col;
	//relative to the world coordinate
	//3 up
	//1 down
	//0 left
	//4 right
	return row_diff+2*col_diff+2;
}

bool updateTargetVertexIndex(uint8_t current_dir, uint8_t traffic_dir, vertexIndex_t& target_vertex_index)
{
	if(traffic_dir == DIR_INVALID) //no traffic sign or no sign has been detected yet
	{
		if(target_vertex_index.row == 0 && target_vertex_index.col==COLS-1) //lower-right
		{
			if(current_dir==4)//right
				target_vertex_index.row++;
			else //down
				target_vertex_index.col--;
		}
		else if(target_vertex_index.row == ROWS-1 && target_vertex_index.col==COLS-1) //upper-right
		{
			if(current_dir==3) //up
				target_vertex_index.col--;
			else //right
				target_vertex_index.row--;
		}
		else //upper-left  pass  //lower-left  pass
			return false;
		return true;
	}

	uint8_t result = current_dir*3 + traffic_dir;
	if(result==2 || result==9 || result== 13) //row++
		target_vertex_index.row++;
	else if(result==1 || result==3 ||result==14) //row--
		target_vertex_index.row--;
	else if(result==4 || result==11 || result==12) //col++
		target_vertex_index.col ++;
	else if(result==0 || result==5 ||result==10) //col--
		target_vertex_index.col--;
	return true;
}

bool dumpPathPoints(const std::string& file_path, const std::vector<gpsMsg_t>& path_points)
{
	FILE *fp = fopen(file_path.c_str(),"w");
	
	if(fp==NULL)
	{
		printf("open %s failed",file_path.c_str());
		return false;
	}
	
	for(size_t i=0;i<path_points.size();i++)
		fprintf(fp,"%lf\t%lf\n",path_points[i].longitude, path_points[i].latitude);
			
	fclose(fp);
	
	return true;
}


void generateNewPathPoints(uint8_t& traffic_dir, std::vector<gpsMsg_t>& path_points)
{
	vertexIndex_t temp = target_vertex_index_;
	
	gpsMsg_t start_point = path_vertexes_[target_vertex_index_.row][target_vertex_index_.col];
	
	uint8_t current_dir = generateCurrentDir(target_vertex_index_,last_vertex_index_);
	
	printf("current_dir:%d",current_dir);
	
	if(!updateTargetVertexIndex(current_dir, traffic_dir, target_vertex_index_))
	{
		printf("updateTargetVertexIndex failed");
		return;
	}
		
	if(target_vertex_index_.row >= ROWS || target_vertex_index_.col >= COLS)
	{
		printf("target_vertex_index_ error");
		return;
	}
	
	gpsMsg_t end_point = path_vertexes_[target_vertex_index_.row][target_vertex_index_.col]; 
			
	size_t points_cnt = disBetween2Points(start_point,end_point)/DIS_INCREMENT;
	
	printf("start_point: %f\t%f\t end_point: %f\t%f\t points_cnt:%d",
			start_point.longitude,start_point.latitude,end_point.longitude,end_point.latitude,points_cnt);
	
	double longitude_increment = (end_point.longitude - start_point.longitude)/points_cnt;
	double latitude_increment  = (end_point.latitude - start_point.latitude)/points_cnt;
	for(size_t i=0; i<points_cnt; i++)
	{
		gpsMsg_t point;
		point.longitude = start_point.longitude + longitude_increment *i;
		point.latitude = start_point.latitude + latitude_increment *i;
		path_points.push_back(point);
	}
	last_vertex_index_ = temp;
}

bool loadPathVertexes(const std::string& file_path,  gpsMsg_t path_vertexes[ROWS][COLS])
{
	FILE *fp = fopen(file_path.c_str(),"r");
	if(fp==NULL)
	{
		printf("open %s failed",file_path.c_str());
		return false;
	}
	gpsMsg_t point;
	int i=0;
	
	while(!feof(fp) && i< ROWS*COLS)
	{
		fscanf(fp,"%lf\t%lf\t",&point.longitude,&point.latitude);
		path_vertexes[0][i] = point;
		i++;
	}
	fclose(fp);
	
	if(i != ROWS*COLS)
	{
		printf("path vertex count error!!!");
		return false;
	}
	return true;
}

int main()
{
	if(!loadPathVertexes("data.txt", path_vertexes_))
		return false;
	
	if(!generateOriginPathPoints(path_points_))
		return false;
	
	traffic_dir = 2;
	
	generateNewPathPoints(traffic_dir,path_points_);
	
	traffic_dir = DIR_INVALID;
	
	generateNewPathPoints(traffic_dir,path_points_);
	
	traffic_dir = 1;
	
	generateNewPathPoints(traffic_dir,path_points_);
	
	traffic_dir = 2;
	
	generateNewPathPoints(traffic_dir,path_points_);
	
	traffic_dir = 2;
	
	generateNewPathPoints(traffic_dir,path_points_);
	
	traffic_dir = 1;
	
	generateNewPathPoints(traffic_dir,path_points_);
	
	traffic_dir = 255;
	
	generateNewPathPoints(traffic_dir,path_points_);
	
	traffic_dir = 2;
	
	generateNewPathPoints(traffic_dir,path_points_);
	
	dumpPathPoints("debug.txt",path_points_);
}
