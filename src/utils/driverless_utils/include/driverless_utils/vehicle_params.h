#ifndef VEHICLE_PARAMS_H_
#define VEHICLE_PARAMS_H_

#define ZERO_CAR 0
#define LITTLE_ANT 1

#define VEHICLE_MODEL LITTLE_ANT

#if VEHICLE_MODEL==ZERO_CAR

	#define MAX_STEERING_ANGLE 25.0
	#define MAX_ROAD_WHEEL_ANGLE 25.0
	
	#define AXIS_DISTANCE   0.7

	const float max_side_acceleration = 1.9; // m/s/s
	
#elif VEHICLE_MODEL==LITTLE_ANT
	#define MAX_STEERING_ANGLE 450.0
	#define MAX_ROAD_WHEEL_ANGLE 20.0
	
	#define AXIS_DISTANCE   1.5

	const float max_side_acceleration = 1.9; // m/s/s

#endif

//方向盘最大转角/前轮最大转角
float g_steering_gearRatio = MAX_STEERING_ANGLE/MAX_ROAD_WHEEL_ANGLE;










#endif
