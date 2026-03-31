#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"
#include <cmath>
#include <algorithm>

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    /////////////////////////////////
    /// TODO: You can add any [private] member variable or [private] member function you need.
    /////////////////////////////////

    // Check if two circles moving with constant velocity will collide
    bool will_collide(const Vec &pos1, const Vec &v1, double r1,
                      const Vec &pos2, const Vec &v2, double r2, double time_interval) {
        Vec delta_pos = pos1 - pos2;
        Vec delta_v = v1 - v2;
        double sum_r = r1 + r2;

        // If velocities are the same, just check current distance
        if (delta_v.norm_sqr() < 1e-9) {
            return delta_pos.norm_sqr() < sum_r * sum_r - 0.01;
        }

        // Check if robots are moving apart
        double project = delta_pos.dot(delta_v);
        if (project >= 0) {
            return false;
        }

        // Find closest approach distance
        project /= -delta_v.norm();
        double min_dis_sqr;

        if (project < delta_v.norm() * time_interval) {
            // Closest point is within the time interval
            min_dis_sqr = delta_pos.norm_sqr() - project * project;
        } else {
            // Closest point is after the time interval
            min_dis_sqr = (delta_pos + delta_v * time_interval).norm_sqr();
        }

        return min_dis_sqr <= sum_r * sum_r - 0.01;
    }

    // Calculate preferred velocity (directly toward target)
    Vec get_preferred_velocity() {
        Vec to_target = pos_tar - pos_cur;
        double dist = to_target.norm();

        if (dist < 0.01) {
            return Vec(0, 0);
        }

        // Move at max speed toward target, or slow down when close
        double desired_speed = std::min(v_max, dist / 0.1);
        return to_target.normalize() * desired_speed;
    }

public:

    Vec get_v_next() {
        /// TODO: You need to decide the speed of the robot at the next moment.
        ///       You can obtain information about the robot being processed in the member variable of class Controller.
        ///       You can obtain information about the other robot by accessing the interface of *monitor.
        /// Warning: You cannot use any static variable or global variable!
        ///          You should not try to output any information!
        ///          You cannot modify any code that is not allowed to be modified!
        ///          All illegal behavior will be voided.

        const double TIME_INTERVAL = 0.1;
        int num_robots = monitor->get_robot_number();

        // Calculate preferred velocity (toward target)
        Vec preferred_v = get_preferred_velocity();

        // If already at target, stay still
        if ((pos_tar - pos_cur).norm() < 0.01) {
            return Vec(0, 0);
        }

        // Try preferred velocity first
        bool collision_free = true;
        for (int i = 0; i < num_robots; i++) {
            if (i == id) continue;

            Vec other_pos = monitor->get_pos_cur(i);
            Vec other_v = monitor->get_v_cur(i);
            double other_r = monitor->get_r(i);

            if (will_collide(pos_cur, preferred_v, r, other_pos, other_v, other_r, TIME_INTERVAL)) {
                collision_free = false;
                break;
            }
        }

        if (collision_free) {
            return preferred_v;
        }

        // If preferred velocity causes collision, try velocity obstacle avoidance
        Vec best_v = Vec(0, 0);
        double best_score = -1e9;

        // Sample different velocities
        int num_samples = 36;  // Sample in 36 directions
        int speed_samples = 5;  // Sample at different speeds

        for (int angle_idx = 0; angle_idx < num_samples; angle_idx++) {
            double angle = 2.0 * 3.14159265358979323846 * angle_idx / num_samples;
            Vec direction(std::cos(angle), std::sin(angle));

            for (int speed_idx = 0; speed_idx <= speed_samples; speed_idx++) {
                double speed = v_max * speed_idx / speed_samples;
                Vec candidate_v = direction * speed;

                // Check if this velocity causes collision
                bool safe = true;
                for (int i = 0; i < num_robots; i++) {
                    if (i == id) continue;

                    Vec other_pos = monitor->get_pos_cur(i);
                    Vec other_v = monitor->get_v_cur(i);
                    double other_r = monitor->get_r(i);

                    if (will_collide(pos_cur, candidate_v, r, other_pos, other_v, other_r, TIME_INTERVAL)) {
                        safe = false;
                        break;
                    }
                }

                if (!safe) continue;

                // Score this velocity
                Vec to_target = pos_tar - pos_cur;
                double progress = candidate_v.dot(to_target.normalize());
                double speed_factor = speed / v_max;
                double score = progress * 10.0 + speed_factor * 1.0;

                if (score > best_score) {
                    best_score = score;
                    best_v = candidate_v;
                }
            }
        }

        return best_v;
    }
};


/////////////////////////////////
/// TODO: You can add any class or struct you need.
/////////////////////////////////


#endif //PPCA_SRC_HPP
