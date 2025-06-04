#include <iostream>
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;
using namespace Eigen;

constexpr  double v_r = 0.4; // reference linear velocity
constexpr  double w_r = 0.5; // reference angular velocity
constexpr  double radius = 2.0; // radius of the circle
constexpr double dt = 0.01; // time step
constexpr double sim_time = 10.0; // total simulation time
constexpr int steps = static_cast<int>(sim_time / dt); // number of steps

MatrixXd SolveLQR(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R){
    // solve Algebraic Riccati equation: A'P + PA - PBR^(-1)B'P + Q = 0
    MatrixXd P = Q;
    for(int i = 0; i < 100; i++){
        MatrixXd P_next = Q + A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
        if((P_next - P).norm() < 1e-6){
            break;
        }
        cout<<"Iteration " << i << ": P norm = " << (P_next - P).norm() << endl;
        P = P_next;
    }
    // optimal gain matrix K = R^(-1)B'P
    return (R + B.transpose() * P * B).inverse() * B.transpose() * P;
}

int main(){
    // state space model
    Matrix3d A;
    A << 0, w_r, 0,
         -w_r, 0, v_r,
         0, 0, 0;
    MatrixXd B(3, 2);
    B << 1, 0,
         0, 0,
         0, 1;

    // LQR parameters
    Matrix3d Q;
    Q << 10, 0, 0,
         0, 10, 0,
         0, 0, 5; // state cost
    Matrix2d R;
    R << 1, 0,
         0, 1; // control cost
    Matrix<double, 2, 3> K = SolveLQR(A, B, Q, R);
    cout<<"K matrix:\n" << K << endl;
    // state initialization
    Vector3d x_e(0.5, 0.2, 0.1); // initial error state
    vector<cv::Point> trajectory, reference_trajectory;

    // graphics setup
    int size = 600;
    int origin = size >> 1;
    double scale = 100.0; // 1m = 100 pixels
    Mat canvas(size, size, CV_8UC3, Scalar(255, 255, 255));

    // simulation loop
    for(int i = 0; i < steps; i++) {
        double t = i * dt;
        double x_r = radius * cos(w_r * t);
        double y_r = radius * sin(w_r * t);
        double theta_r = w_r * t;

        // control law
        Vector2d u = -K * x_e;

        // update error state
        Vector3d dx_e = A * x_e + B * u;
        x_e += dx_e * dt;

        // convert error to global coordinates
        Matrix2d R_theta;
        R_theta << cos(theta_r), -sin(theta_r),
                sin(theta_r), cos(theta_r);
        Vector2d pe = R_theta * x_e.head<2>();

        double x = x_r + pe(0);
        double y = y_r + pe(1);

        // store trajectory points
        cv::Point p(static_cast<int>(origin + x * scale), static_cast<int>(origin - y * scale));
        trajectory.push_back(p);
        reference_trajectory.emplace_back(static_cast<int>(origin + x_r * scale),
                                          static_cast<int>(origin - y_r * scale));
    }
    // draw trajectory
    canvas = Scalar(255, 255, 255); // clear canvas
    for(int i = 0; i < trajectory.size(); i++){
        // draw start point
        if(i == 0) {
            circle(canvas, trajectory[i], 5, Scalar(0, 255, 0), -1); // start point in green
        }
        if(i == trajectory.size() - 1) {
            circle(canvas, trajectory[i], 5, Scalar(0, 0, 255), -1); // end point in red
        }
        circle(canvas, trajectory[i], 1, Scalar(211, 85, 186), -1);
        circle(canvas, reference_trajectory[i], 1, Scalar(255, 0, 0), -1);

        if(i != 0 && i != trajectory.size() - 1 && i % 20 == 0)
            circle(canvas, trajectory[i], 5, Scalar(128, 128, 240), 1); // draw current position

        // display the canvas
        imshow("LQR Trajectory", canvas);
        char key = (char)waitKey(1);
        if(key == 27) { // ESC key to exit
            break;
        }
    }

    // wait for a key press before closing
    waitKey(0);
    return 0;
}