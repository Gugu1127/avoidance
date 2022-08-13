#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>
#include <string.h>

#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

using namespace std;

#define obstacle_size 5
#define top_distance_stop 0.25
#define top_distance_slow 0.8
#define bottom_distance_stop 0.25
#define bottom_distance_slow 0.8
#define top_corner_distance_stop 0.20
#define bottom_corner_distance_stop 0.20

enum Position {
    top,
    top_R,
    top_L,
    bottom,
    bottom_R,
    bottom_L,
    top_slow,
    bottom_slow
};

class Group {
   private:
    vector<int> index_group[8];
    vector<double> scan_array;
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    void grouping(int start_index, int end_index, vector<int>& target) {
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
        grouping(674, 45, index_group[top_slow]);
        grouping(315, 405, index_group[bottom_slow]);

        sub = n.subscribe<sensor_msgs::LaserScan>("/scan/noise_filter", 1, &Group::scanCallback, this);
        pub = n.advertise<std_msgs::Int16MultiArray>("/collision", 1);
    }

    void get_scan(const sensor_msgs::LaserScan::ConstPtr& scan) {
        scan_array.assign(scan->ranges.begin(), scan->ranges.end());
    }

    void distribute(vector<double> pieces[8], const vector<int> index_array[8]) {
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < index_array[i].size(); j++) {
                pieces[i].push_back(scan_array[index_array[i][j]]);
            }
        }
    }

    bool isCollision(const vector<double>& pieces, int startIndex, float distance) {
        for (int i = 0; i < obstacle_size; i++) {
            if (pieces[i + startIndex] > distance) {
                return false;
            }
        }

        return true;
    }

    int detectCollision(const vector<double>& pieces, float sensing_distance) {
        for (int i = 0; i < pieces.size() - obstacle_size; i++) {
            if (isCollision(pieces, i, sensing_distance)) {
                return 0;
            }
        }

        return 1;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        vector<double> pieces[8];
        get_scan(scan);
        distribute(pieces, index_group);
        std_msgs::Int16MultiArray msg;
        msg.data = {0, 0, 0, 0, 0, 0, 0, 0};
        msg.data[top] = detectCollision(pieces[top], top_distance_stop);
        msg.data[bottom] = detectCollision(pieces[bottom], bottom_distance_stop);
        msg.data[top_L] = detectCollision(pieces[top_L], top_corner_distance_stop);
        msg.data[top_R] = detectCollision(pieces[top_R], top_corner_distance_stop);
        msg.data[bottom_L] = detectCollision(pieces[bottom_L], bottom_corner_distance_stop);
        msg.data[bottom_R] = detectCollision(pieces[bottom_R], bottom_corner_distance_stop);
        msg.data[top_slow] = detectCollision(pieces[top_slow], top_distance_slow);
        msg.data[bottom_slow] = detectCollision(pieces[bottom_slow], bottom_distance_slow);
        pub.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "avoidance");
    Group group;

    ros::spin();

    return 0;
}