#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"
#include <cmath>
#include <algorithm>
#include <vector>

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

    // Calculate preferred velocity (directly toward target)
    Vec get_preferred_velocity() {
        Vec to_target = pos_tar - pos_cur;
        double dist = to_target.norm();

        if (dist < 0.01) {
            return Vec(0, 0);
        }

        // Move at max speed toward target, or slow down when very close
        double desired_speed = std::min(v_max, dist / 0.1 * 1.2);
        return to_target.normalize() * desired_speed;
    }

    // Check if a velocity is safe (no collisions)
    bool is_safe_velocity(const Vec &test_v, double time_interval) {
        int num_robots = monitor->get_robot_number();

        for (int i = 0; i < num_robots; i++) {
            if (i == id) continue;

            Vec other_pos = monitor->get_pos_cur(i);
            Vec other_v = monitor->get_v_cur(i);
            double other_r = monitor->get_r(i);

            Vec delta_pos = pos_cur - other_pos;
            Vec delta_v = test_v - other_v;
            double sum_r = r + other_r + 0.01;  // Minimal safety margin

            // If no relative velocity, just check distance
            if (delta_v.norm_sqr() < 1e-9) {
                if (delta_pos.norm_sqr() < sum_r * sum_r) {
                    return false;
                }
                continue;
            }

            // Check if robots are moving apart
            double project = delta_pos.dot(delta_v);
            if (project >= 0) {
                continue;
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

            if (min_dis_sqr <= sum_r * sum_r) {
                return false;
            }
        }

        return true;
    }

    // ORCA-like velocity computation
    Vec compute_orca_velocity(const Vec &pref_v, double time_interval) {
        int num_robots = monitor->get_robot_number();

        // Collect velocity obstacles
        std::vector<Vec> vo_normals;
        std::vector<double> vo_offsets;

        for (int i = 0; i < num_robots; i++) {
            if (i == id) continue;

            Vec other_pos = monitor->get_pos_cur(i);
            Vec other_v = monitor->get_v_cur(i);
            double other_r = monitor->get_r(i);

            Vec delta_pos = other_pos - pos_cur;
            double dist = delta_pos.norm();
            double sum_r = r + other_r + 0.01;

            // If too close, add strong repulsion
            if (dist < sum_r * 1.4) {
                Vec repulsion = (pos_cur - other_pos).normalize();
                vo_normals.push_back(repulsion);
                vo_offsets.push_back(sum_r * 2.0 - dist);
                continue;
            }

            // ORCA constraint: assume other robot will avoid 50% of collision
            Vec relative_pos = delta_pos;
            Vec relative_vel = v_cur - other_v;

            double tau = 2.0;  // Time horizon for collision avoidance
            Vec u = relative_pos / tau - relative_vel;

            if (u.norm_sqr() < 1e-9) continue;

            Vec normal = u.normalize();
            double offset = u.norm() - (sum_r + 0.01) / tau;

            if (offset < 0) {
                vo_normals.push_back(normal);
                vo_offsets.push_back(-offset);
            }
        }

        // Find velocity closest to preferred velocity while satisfying constraints
        Vec best_v = pref_v;

        // If preferred velocity is safe, use it
        if (is_safe_velocity(pref_v, time_interval)) {
            return pref_v;
        }

        // Otherwise, project away from velocity obstacles
        Vec adjusted_v = pref_v;
        for (size_t i = 0; i < vo_normals.size(); i++) {
            double violation = vo_normals[i].dot(adjusted_v) - vo_offsets[i];
            if (violation < 0) {
                adjusted_v = adjusted_v + vo_normals[i] * (-violation);
            }
        }

        // Clamp to max velocity
        if (adjusted_v.norm() > v_max) {
            adjusted_v = adjusted_v.normalize() * v_max;
        }

        // If adjusted velocity is safe, use it
        if (is_safe_velocity(adjusted_v, time_interval)) {
            return adjusted_v;
        }

        // Fallback: sample velocities
        best_v = Vec(0, 0);
        double best_score = -1e9;

        int num_samples = 48;
        int speed_samples = 6;

        for (int angle_idx = 0; angle_idx < num_samples; angle_idx++) {
            double angle = 2.0 * 3.14159265358979323846 * angle_idx / num_samples;
            Vec direction(std::cos(angle), std::sin(angle));

            for (int speed_idx = 0; speed_idx <= speed_samples; speed_idx++) {
                double speed = v_max * speed_idx / speed_samples;
                Vec candidate_v = direction * speed;

                if (!is_safe_velocity(candidate_v, time_interval)) {
                    continue;
                }

                // Score: prefer velocities close to preferred velocity
                double diff = (candidate_v - pref_v).norm();
                double score = -diff + speed * 0.1;

                if (score > best_score) {
                    best_score = score;
                    best_v = candidate_v;
                }
            }
        }

        return best_v;
    }

public:

    Vec get_v_next() {
        const double TIME_INTERVAL = 0.1;

        // If already at target, stay still
        if ((pos_tar - pos_cur).norm() < 0.01) {
            return Vec(0, 0);
        }

        // Calculate preferred velocity
        Vec pref_v = get_preferred_velocity();

        // Use ORCA-like approach
        Vec result_v = compute_orca_velocity(pref_v, TIME_INTERVAL);

        return result_v;
    }
};


/////////////////////////////////
/// TODO: You can add any class or struct you need.
/////////////////////////////////


#endif //PPCA_SRC_HPP
