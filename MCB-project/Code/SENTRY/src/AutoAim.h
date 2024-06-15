#pragma once

#ifndef AUTOAIM_H_
#define AUTOAIM_H_

#include <cmath>
#include <vector>
#include "tap/algorithms/ballistics.hpp"
#include "JetsonCommunication.h"
#include "modm/math/geometry/vector.hpp"


namespace ThornBots {
    using namespace tap::algorithms::ballistics;
    class AutoAim {
    public:
        AutoAim() = default;
        ~AutoAim() = default;
        struct PanelData {
            double r;
            double theta;
        };

        void update(JetsonCommunication::cord_msg* msg, float current_pitch, float current_yaw, double& yawOut, double& pitchOut, int& action) {
            // Add rotated offset vector of panel relative to RGB
            if (msg->z == 0.0f) return;
            // if (msg->z == 0) return;  FIXME:<---- this breaks?
        
            // float X_prime = -x + 0.0175;                                                     // left
            // float Y_prime = -y + 0.1295 * cos(current_pitch) - 0.0867 * sin(current_pitch);  // up
            // float Z_prime = z + 0.0867 * cos(current_pitch) + 0.1295 * sin(current_pitch);   // forwards


            //TODO: just store vec3s in JetosnComms msg struct
            // modm::Vector3f pos(msg->x,msg->y,msg->z);
            // modm::Vector3f vel(msg->v_x,msg->v_y,msg->v_z);
            // modm::Vector3f acc(msg->a_x,msg->a_y,msg->a_z);


            // MeasuredKinematicState state;//(pos,vel,acc); 
            // state.position = modm::Vector3f(0,0,0);
            // state.velocity = modm::Vector3f(0,0,0);
            // state.acceleration = modm::Vector3f(0,0,0);

            MeasuredKinematicState state;//(pos,vel,acc); 
            state.position = modm::Vector3f(msg->x,msg->y,msg->z);
            state.velocity = modm::Vector3f(msg->v_x,msg->v_y,msg->v_z);
            state.acceleration = modm::Vector3f(msg->a_x,msg->a_y,msg->a_z);

            float targetYaw, targetPitch, travelTime;
            bool valid = tap::algorithms::ballistics::findTargetProjectileIntersection(state, J, 3, &targetPitch, &targetYaw, &travelTime, 0);

            if(!valid){
                action = -1;// make enums for action
                return;
            }

            yawOut = fmod(current_yaw + targetYaw, 2 * PI);
            pitchOut = targetPitch;

            if (abs(targetYaw) < 5 * M_PI / 180) {
                // Enable shooting
                action = 1;
                return;
            }
            action = 0;

            //TODO:================================================================================ 

            // // Convert to cylindrical coordinates
            // double r_prime, theta_prime, Z_double_prime;
            // cartesianToCylindrical(X_prime, Y_prime, Z_prime, r_prime, theta_prime, Z_double_prime);

            // // Check if the target is above the height rejection offset
            // //  if (Z_double_prime > H) {
            // //      action = -1;
            // //      return;
            // //  }

            // // Update with the new panel data
            // panelData.push_back({r_prime, theta_prime});
            // if (panelData.size() > 3) panelData.erase(panelData.begin());

            // // Compute finite differences for velocity and acceleration
            // double dr_dt = 0, d2r_dt2 = 0, dp_dt = 0, d2p_dt2 = 0;
            // computeFiniteDifferences(panelData, deltaTime, dr_dt, d2r_dt2, dp_dt, d2p_dt2);

            // // Calculate shot timing
            // double deltaT_shot = (-dr_dt - 1) / d2r_dt2 + sqrt(pow(dr_dt / J - 1, 2) - 2 * d2r_dt2 / J * (r_prime / J + l)) / (2 * d2r_dt2 / J);

            // // Compute the target panel position at impact
            // double r_triple_prime = r_prime + dr_dt * deltaT_shot + d2r_dt2 * pow(deltaT_shot, 2) / 2;
            // double theta_triple_prime = theta_prime + dp_dt * deltaT_shot + d2p_dt2 * pow(deltaT_shot, 2) / 2;

            // // Send this position and velocity to the turret controller
            // yawOut = fmod(theta_triple_prime + current_yaw, 2 * M_PI);
            // // lets not set yaw prime yet. This should make the controller less aggressive for now

            // // Check if the yaw angle is within the threshold
            // if (abs(theta_triple_prime) < 5 * M_PI / 180) {
            //     // Enable shooting
            //     action = 1;
            // }

            // // Bullet drop calculations
            // pitchOut = asin((Z_double_prime + g * pow(deltaT_shot, 2) / 2) / (J * deltaT_shot));
            // Send s_prime to pitch controller
        }

    private:
        // Constants
        const float g = 9.81;           // gravitational acceleration
        const float J = 25.0;           // Shot velocity
        const float l = 0.05;           // Combined camera + Jetson latency
        const float deltaTime = 0.033;  // Frame time
        const float H = 0.35;           // Height rejection offset
        std::vector<PanelData> panelData;

        void cartesianToCylindrical(double x, double y, double z, double& r, double& theta, double& Z_double_prime) {
            r = sqrt(x * x + z * z);
            theta = atan2(x, z);
            Z_double_prime = y;
        }

        void computeFiniteDifferences(const std::vector<PanelData>& data, double deltaTime, double& dr_dt, double& d2r_dt2, double& dp_dt,
                                      double& d2p_dt2) {
            size_t n = data.size();
            if (n > 2) {
                dr_dt = (data[n - 3].r - 4 * data[n - 2].r + 3 * data[n - 1].r) / (2 * deltaTime);
                d2r_dt2 = -(data[n - 3].r + 2 * data[n - 2].r - data[n - 1].r) / (deltaTime * deltaTime);
                dp_dt = (data[n - 3].theta - 4 * data[n - 2].theta + 3 * data[n - 1].theta) / (2 * deltaTime);
                d2p_dt2 = -(data[n - 3].theta + 2 * data[n - 2].theta - data[n - 1].theta) / (deltaTime * deltaTime);
            }
        }
    };
}  // namespace ThornBots
#endif  // DRIVERS_SINGLETON_HPP_