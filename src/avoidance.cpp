#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>

#include <iostream>
#include <string>
#include <vector>
#include <fstream>

using namespace std;

#define default_window_size 1
#define obstacle_size 5
#define top_distance_stop 0.2
#define top_distance_slow 0.6
#define bottom_distance_stop 0.25
#define bottom_distance_slow 0.6
#define top_corner_distance_stop 0.20
#define bottom_corner_distance_stop 0.20

int window_size = default_window_size ;
int counter = 0 ;


enum Position {
    top,
    top_R,
    top_L,
    bottom,
    bottom_R,
    bottom_L
};

class Group {
   private:
    vector<int> index_group[6];
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Subscriber sub_window;
    ros::Publisher pub;

    void grouping(int start_index, int end_index, vector<int> &target) {
        target.clear();
        int currentIndex = start_index;
        while (currentIndex != end_index) {
            target.push_back(currentIndex);
            currentIndex += 1;
            if (currentIndex >= 720) {
                currentIndex = 0;
            }
        }
    }

   public:
    Group() {
        
        grouping(635, 84, index_group[top]);
        grouping(60, 179, index_group[top_L]);
        grouping(540, 659, index_group[top_R]);
        grouping(300, 419, index_group[bottom]);
        grouping(195, 284, index_group[bottom_L]);
        grouping(435, 524, index_group[bottom_R]);

        sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Group::scanCallback, this);
        sub_window = n.subscribe("/sliding_window/set", 1, &Group::set_window_size, this);
        pub = n.advertise<std_msgs::Int16MultiArray>("/collision", 1);
    }

    void set_window_size(const std_msgs::Int8& msg) {
        int temp = window_size;
        window_size += msg.data ;
        if (window_size < 0) {
            window_size = temp;
        }

        //cout << "Current window size: " << window_size << endl;
    }

    void filter(vector<double>& average_array, const sensor_msgs::LaserScan::ConstPtr& scan) {
        average_array.clear();
        int half_window_size = floor(window_size / 2);
        for (int i = 0; i < 720 ; i++) {
            double total = 0, no_zero_elements = 0;
            for (int j = i - half_window_size; j <= i + half_window_size ; j++) {
                int index = j;
                if ( j < 0) {
                    index += 720;
                } else if (j >= 720) {
                    index -= 720;
                }

                total += scan->ranges[index] ;
                if (scan->ranges[index] > 0) {
                    no_zero_elements++;
                }
            }

            average_array.push_back(no_zero_elements == 0 ? 0 : total / no_zero_elements);
        }

        if (counter == 100) {
            ROS_INFO("current window size: %d", window_size);
            window_size += 2;
            cout << endl;
            counter = 0;
        }

        if (counter == 0) {
            cout << "window_size = " << window_size << ", ";
        }

        int num_of_zeros = 0;
        for (int i = 0 ; i < average_array.size() ; i++) {
            if (average_array[i] == 0) {
                num_of_zeros++ ;   
            }
        }


        cout << num_of_zeros << ", ";

        counter += 1;
    }

    void distribute(vector<double> array[6], const vector<int> index_array[6], const vector<double>& average_array) {
        for (int i = 0 ; i < 6 ; i++) {
            for (int j = 0 ; j < index_array[i].size() ; j++) {
                array[i].push_back(average_array[index_array[i][j]]);
            }
        }
    }

    bool isCollision(const vector<double> &average_array, int startIndex, float distance) {
        for (int i = 0 ; i < obstacle_size ; i++) {
            if (i + startIndex >= average_array.size()) {
                return false;
            }

            if (average_array[i + startIndex] > distance) {
                return false;
            }
        }

        return true;
    }

    int detectCollision(const vector<double> &average_array, float distance_stop, float distance_slow) {
        for (int i = 0 ; i < average_array.size() ; i++) {
            if (isCollision(average_array, i, distance_stop)) {
                return 0;
            }
        }

        for (int i = 0 ; i < average_array.size() ; i++) {
            if (isCollision(average_array, i, distance_slow)) {
                return 2;
            }
        }

        return 1;
    }

    int detectCollision(const vector<double> &average_array, float distance_stop) {
        for (int i = 0 ; i < average_array.size() ; i++) {
            if (isCollision(average_array, i, distance_stop)) {
                return 0;
            }
        }

        return 1;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        vector<double> filter_array;
        filter(filter_array, scan);
        vector<double> arrayList[6];
        distribute(arrayList, index_group, filter_array);

        std_msgs::Int16MultiArray msg;
        msg.data={0,0,0,0,0,0};
        msg.data[top]=detectCollision(arrayList[top], top_distance_stop, top_distance_slow);
        msg.data[bottom]=detectCollision(arrayList[bottom], bottom_distance_stop, bottom_distance_slow);
        msg.data[top_L]=detectCollision(arrayList[top_L], top_corner_distance_stop);
        msg.data[top_R]=detectCollision(arrayList[top_R], top_corner_distance_stop);
        msg.data[bottom_L]=detectCollision(arrayList[bottom_L], bottom_corner_distance_stop);
        msg.data[bottom_R]=detectCollision(arrayList[bottom_R], bottom_corner_distance_stop);
        pub.publish(msg);


    }
};

int main(int argc, char** argv) {
    fstream file("/home/ros/catkin_ws/src/avoidance/output.csv", ios::out);
    cout.rdbuf(file.rdbuf());
    ros::init(argc, argv, "avoidance");
    Group group;

    ros::spin();

    return 0;
}