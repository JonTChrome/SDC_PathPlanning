#ifndef UTILITIES
#define UTILITIES

#define FOLLOW_DISTANCE 30 //how far away we want to consider decelerating

#define ACCEL_INCREMENT 0.04

#define MIN_FOLLOW_DISTANCE 12 // minimum following distance

#define PATH_SIZE 50
#define DT 0.02 //Seconds per each point

#define MAX_ACCEL 10 //m/s^2
#define MAX_JERK 10 //m/s^3
#define SPEED_DIVISOR 10
#define DESIRED_VEL  49.5/SPEED_DIVISOR

#define DIF_SPEED_WEIGHT 1000
#define DIF_DIST_WEIGHT 1000

#define MIN_DISTANCE_BEHIND 20
#define MIN_DISTANCE_AHEAD 20

  //Try to change lanes if we are 20% below the desired velocity
#define TRY_CHANGE_LANE_VEL  DESIRED_VEL - DESIRED_VEL *0.2

using namespace std;
//Finite States
enum CarState { maintain, accelerate, deccelerate, change_lane_right, change_lane_left, try_change_lanes, changing_lanes};

//We now want to change lanes, which lane should we choose?
CarState check_which_lane(vector<vector<double>> sensor_fusion, int lane, int path_size, double car_s, double &current_vel, double &decel_vel) {
	CarState returnState = maintain;
	bool foundPath = false;
	double cur_cost = 50000;
	int newLane = lane;
	double cost;
	for(int i = 0; i < sensor_fusion.size(); i++) {
        float d = sensor_fusion[i][6];
        //car is outside my lane
        if((d > 0) && (d > (2 + 4 * lane + 2) || d < (2 + 4 * lane - 2))) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double) path_size * .02 * check_speed);
            double dist_diff = abs(check_car_s - car_s);
            cout << "Car_s: " << car_s << endl;
            cout << "check car s: " << check_car_s << endl; 
            if(!(check_car_s < car_s - MIN_DISTANCE_BEHIND) && (check_car_s > car_s + MIN_DISTANCE_AHEAD)) {
             	cost = abs(check_speed - current_vel) * DIF_SPEED_WEIGHT + abs(check_car_s - car_s) * DIF_DIST_WEIGHT;
            	cout << "cost: " << cost << endl;
            	if (cost < cur_cost) {
            		foundPath = true;
            		cur_cost = cost;
            		newLane = (d - 2)/4;
            		if(newLane > lane) {
            			returnState = change_lane_right;
            		} else if (newLane < lane) {
            			returnState = change_lane_left;
            		}
            	}
            }

        }
	}
	if(foundPath) {
		cout << "STATE FOUND" << endl;
		cout << "cost: " << cost << endl;
		return returnState;
	} else {
			//failed state
		returnState = deccelerate;
		cout << "NO LANE CHANGE FOUND " << endl;
		return returnState;
	}

}

//Have we completed our lane change?
CarState check_lane_change_complete(int lane, double car_d) {
	if(car_d < 2 + 4 * lane + 2 && car_d > 2 + 4 * lane - 2) {
		return maintain;
	} 
	return changing_lanes;
}

//main fsm transition function
CarState check_lanes(vector<vector<double>> sensor_fusion, int lane, int path_size, double car_s, CarState current, double &current_vel, double &decel_vel) {
	CarState returnState = accelerate;

	if(current == try_change_lanes) {
			returnState = check_which_lane(sensor_fusion, lane, path_size, car_s, current_vel, decel_vel);
			return returnState;
	} else {
		for(int i = 0; i < sensor_fusion.size(); i++) {
	        float d = sensor_fusion[i][6];
	        //car is in my lane
	        if(d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
	            double vx = sensor_fusion[i][3];
	            double vy = sensor_fusion[i][4];
	            double check_speed = sqrt(vx*vx + vy*vy);
	            double check_car_s = sensor_fusion[i][5];

	            check_car_s += ((double) path_size * .02 * check_speed);
	            double dist_diff = (check_car_s - car_s);
	            if ((check_car_s > car_s) && (dist_diff < MIN_FOLLOW_DISTANCE) && check_speed/SPEED_DIVISOR < TRY_CHANGE_LANE_VEL) {
	            	returnState = deccelerate;
	            	return returnState;
	            } else if((check_car_s > car_s) && (dist_diff < FOLLOW_DISTANCE) && check_speed/SPEED_DIVISOR > TRY_CHANGE_LANE_VEL) {
	            	decel_vel = check_speed / SPEED_DIVISOR;
	            	cout << "decel speed: " << check_speed << " try change lane vel" << TRY_CHANGE_LANE_VEL << endl;
	            	returnState = deccelerate;
	            	return returnState;
	            } else if((check_car_s > car_s) && (dist_diff < FOLLOW_DISTANCE) && check_speed/SPEED_DIVISOR < TRY_CHANGE_LANE_VEL && current != changing_lanes) {
	            	returnState = try_change_lanes;
	            	return returnState;
	            } 
	        }
	    }
	}
    return returnState;
}

#endif