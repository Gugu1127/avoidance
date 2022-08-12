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

#define default_window_size 31
#define obstacle_size 5
#define top_distance_stop 0.2
#define top_distance_slow 0.8
#define bottom_distance_stop 0.25
#define bottom_distance_slow 0.8
#define top_corner_distance_stop 0.20
#define bottom_corner_distance_stop 0.20

#define time_slice 5

int window_size = default_window_size;

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
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Subscriber sub_window;
    ros::Publisher pub;

    vector<vector<double>> queue;

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

        sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Group::scanCallback, this);
        sub_window = n.subscribe("/sliding_window/set", 1, &Group::set_window_size, this);
        pub = n.advertise<std_msgs::Int16MultiArray>("/collision", 1);
    }

    void set_window_size(const std_msgs::Int8& msg) {
        int temp = window_size;
        window_size += msg.data * 100;
        if (window_size < 0) {
            window_size = temp;
        }

        cout << "Current window size: " << window_size << endl;
    }

    void filter(vector<double>& average_array, const sensor_msgs::LaserScan::ConstPtr& scan) {
        average_array.clear();
        int half_window_size = floor(window_size / 2);
        for (int i = 0; i < 720; i++) {
            double total = 0, no_zero_elements = 0;
            for (int j = i - half_window_size; j <= i + half_window_size; j++) {
                int index = j;
                if (j < 0) {
                    index += 720;
                } else if (j >= 720) {
                    index -= 720;
                }

                total += scan->ranges[index];
                if (scan->ranges[index] > 0) {
                    no_zero_elements++;
                }
            }

            average_array.push_back(no_zero_elements == 0 ? 0 : total / no_zero_elements);
        }
    }

    void distribute(vector<double> array[8], const vector<int> index_array[8], const vector<double>& average_array) {
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < index_array[i].size(); j++) {
                array[i].push_back(average_array[index_array[i][j]]);
            }
        }
    }

    bool isCollision(const vector<double>& average_array, int startIndex, float distance) {
        for (int i = 0; i < obstacle_size; i++) {
            if (i + startIndex >= average_array.size()) {
                return false;
            }

            if (average_array[i + startIndex] > distance) {
                return false;
            }
        }

        return true;
    }

    int detectCollision(const vector<double>& average_array, float distance_stop) {
        for (int i = 0; i < average_array.size(); i++) {
            if (isCollision(average_array, i, distance_stop)) {
                return 0;
            }
        }

        return 1;
    }

    double sd(double sd_array[time_slice]) {
        double sum = 0, sum_i;
        for (int i = 0; i < time_slice; i++) {
            sum += sd_array[i];
            sum_i += sd_array[i] * sd_array[i];
        }
        double average = sum / time_slice;
        return sqrt(sum_i / time_slice - average * average);
    }

    int var(double var_array[time_slice]) {
        int count = 0;
        for (int i = 0; i < time_slice - 1; i++) {
            if (var_array[i] == 0 && var_array[i + 1] != 0 || var_array[i] != 0 && var_array[i + 1] == 0) {
                count++;
            }
        }
        return count;
    }

    vector<double> average() {
        double var_array[720];
        int weight[] = {1, 2, 3, 4, 8};
        vector<double> array(720);
        for (int i = 0; i < queue.size(); i++) {
            for (int j = 0; j < queue[i].size(); j++) {
                array[j] += queue[i][j] * weight[i];
            }
        }
        double sum = 0;
        for (int i = 0; i < 720; i++) {
            double array[time_slice];

            for (int j = 0; j < time_slice; j++) {
                array[j] = queue[j][i];
            }

            sum += var(array);
        }
        cout << sum / 720 << endl;
        if (sum / 720 > 0.08) {
            window_size += 20;
        } else {
            window_size -= 10;
        }

        if (window_size < 31) {
            window_size = 31;
        } else if (window_size > 101) {
            window_size = 101;
        }

        cout << "window size: " << window_size << endl;

        for (int i = 0; i < 720; i++) {
            array[i] /= 18;
        }

        queue.erase(queue.begin());
        return array;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        vector<double> filter_array;
        filter(filter_array, scan);
        queue.push_back(move(filter_array));
        if (queue.size() == time_slice) {
            publish_message(move(average()));
        }
    }

    void publish_message(const vector<double>& filter_array) {
        vector<double> arrayList[8];
        distribute(arrayList, index_group, filter_array);
        std_msgs::Int16MultiArray msg;
        msg.data = {0, 0, 0, 0, 0, 0, 0, 0};
        msg.data[top] = detectCollision(arrayList[top], top_distance_stop);
        msg.data[bottom] = detectCollision(arrayList[bottom], bottom_distance_stop);
        msg.data[top_L] = detectCollision(arrayList[top_L], top_corner_distance_stop);
        msg.data[top_R] = detectCollision(arrayList[top_R], top_corner_distance_stop);
        msg.data[bottom_L] = detectCollision(arrayList[bottom_L], bottom_corner_distance_stop);
        msg.data[bottom_R] = detectCollision(arrayList[bottom_R], bottom_corner_distance_stop);
        msg.data[top_slow] = detectCollision(arrayList[top_slow], top_distance_slow);
        msg.data[bottom_slow] = detectCollision(arrayList[bottom_slow], bottom_distance_slow);
        pub.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "avoidance");
    Group group;

    ros::spin();

    return 0;
}