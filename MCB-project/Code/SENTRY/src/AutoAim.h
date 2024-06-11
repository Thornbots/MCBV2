#pragma once

#include <cmath>
#include <vector>

namespace ThornBots {
    class AutoAim {
    public:
        AutoAim() = default;
        ~AutoAim() = default;
        struct PanelData {
            double r;
            double theta;
        };
        struct GimbalCommand {
            double yaw;
            double yaw_prime;
            double pitch;
            int action;  //-1 ignore, 0 follow, 1 to fire
        };
        GimbalCommand update(double x, double y, double z, double current_pitch, double current_yaw) {
            GimbalCommand command = {0, 0, 0, -1};

            // Add rotated offset vector of panel relative to RGB
            double X_prime = x + 0.0175;
            double Y_prime = y + 0.1295 * cos(current_pitch) - 0.0867 * sin(current_pitch);
            double Z_prime = z + 0.0867 * cos(current_pitch) + 0.1295 * sin(current_pitch);

            // Convert to cylindrical coordinates
            double r_prime, theta_prime, Z_double_prime;
            cartesianToCylindrical(X_prime, Y_prime, Z_prime, r_prime, theta_prime, Z_double_prime);

            // Check if the target is above the height rejection offset
            if (Z_double_prime > H) {
                return command;
            }

            // Update with the new panel data
            panelData.push_back({r_prime, theta_prime});
            if (panelData.size() > 3) panelData.erase(panelData.begin());

            // Compute finite differences for velocity and acceleration
            double dr_dt = 0, d2r_dt2 = 0, dp_dt = 0, d2p_dt2 = 0;
            computeFiniteDifferences(panelData, deltaTime, dr_dt, d2r_dt2, dp_dt, d2p_dt2);

            // Calculate shot timing
            double deltaT_shot = (-dr_dt - 1) / d2r_dt2 + sqrt(pow(dr_dt / J - 1, 2) - 2 * d2r_dt2 / J * (r_prime / J + l)) / (2 * d2r_dt2 / J);

            // Compute the target panel position at impact
            double r_triple_prime = r_prime + dr_dt * deltaT_shot + d2r_dt2 * pow(deltaT_shot, 2) / 2;
            double theta_triple_prime = theta_prime + dp_dt * deltaT_shot + d2p_dt2 * pow(deltaT_shot, 2) / 2;

            // Send this position and velocity to the turret controller
            command.yaw = fmod(theta_triple_prime + current_yaw, M_2_PI);
            // lets not set yaw prime yet. This should make the controller less aggressive for now

            // Check if the yaw angle is within the threshold
            if (abs(theta_triple_prime) < 5 * M_PI / 180) {
                // Enable shooting
                command.action = 1;
            }

            // Bullet drop calculations
            command.pitch = asin((Z_double_prime + g * pow(deltaT_shot, 2) / 2) / (J * deltaT_shot));
            // Send s_prime to pitch controller

            return command;
        }

    private:
        // Constants
        const double g = 9.81;           // gravitational acceleration
        const double J = 25.0;           // Shot velocity
        const double l = 0.05;            // Combined camera + Jetson latency
        const double deltaTime = 0.033;  // Frame time
        const double H = 0.35;            // Height rejection offset
        std::vector<PanelData> panelData;

        void cartesianToCylindrical(double x, double y, double z, double& r, double& theta, double& Z_double_prime) {
            r = sqrt(x * x + z * z);
            theta = atan2(z, x);
            Z_double_prime = y;
        }

        void computeFiniteDifferences(const std::vector<PanelData>& data, double deltaTime, double& dr_dt, double& d2r_dt2, double& dp_dt,
                                      double& d2p_dt2) {
            size_t n = data.size();
            if (n > 2) {
                dr_dt = (data[n - 2].r - 4 * data[n - 1].r + 3 * data[n].r) / (2 * deltaTime);
                d2r_dt2 = -(data[n - 2].r + 2 * data[n - 1].r - data[n].r) / (deltaTime * deltaTime);
                dp_dt = (data[n - 2].theta - 4 * data[n - 1].theta + 3 * data[n].theta) / (2 * deltaTime);
                d2p_dt2 = -(data[n - 2].theta + 2 * data[n - 1].theta - data[n].theta) / (deltaTime * deltaTime);
            } else if (n > 1) {
                dr_dt = (data[n].r - data[n - 1].r) / (deltaTime);
                dp_dt = (data[n].theta - data[n - 1].theta) / (deltaTime);
            }
        }
    };
}  // namespace ThornBots