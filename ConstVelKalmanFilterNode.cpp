//Dock node and Filter
#include "ff_estimate/base_mocap_estimator.hpp"


#include <Eigen/Dense>
 
using Eigen::MatrixXd;
using Eigen::VectorXd;
using ff_msgs::msg::FreeFlyerState;
using ff_msgs::msg::Pose2DStamped;
using ff_msgs::msg::Pose2D;

using namespace std;

class ConstVelKalmanFilterNode : public ff::BaseMocapEstimator {

    public:
        Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<6,6>::Identity(6, 6); //Process Noise Covariance Matrix
	// R is Measurement Covariance Matrix. Can be thought of as observation error
        Eigen::Matrix<double, 3, 3> R {
			{2.4445e-3,    0     ,     0    },
			{    0    , 1.2527e-3,     0    },
			{    0    ,    0     , 4.0482e-3},
		};
        double MAX_DT = 1e-3;

        ConstVelKalmanFilterNode() : ff::BaseMocapEstimator("const_vel_kalman_filter_node") {
            this->declare_parameter("min_dt", 0.005);
        }

        void EstimatewithPose2D(const Pose2DStamped & pose_stamped) override {

         //R = Eigen::Matrix<3,3>::Identity(3, 3) * 2.4445e-3, 1.2527e-3, 4.0482e-3;
        
        FreeFlyerState state{};

		state.pose = pose_stamped.pose;
	/* In order to use new state prediction, velocity must be discovered, and because there is constant vel
 	   We can do (current state  - previous state)/ change in time
     	*/
        if (prev_state_ready_) {
            const rclcpp::Time now = pose_stamped.header.stamp;
            const rclcpp::Time last = prev_.header.stamp;
            double dt = (now - last).seconds();

            if (dt < (this->get_parameter("min_dt").as_double())) {
                return;
            }
			//should prev state be taken from pose_stamped or state?
			double vx = (pose_stamped.pose.x - prev_.state.pose.x) / dt;
			double vy = (pose_stamped.pose.y - prev_.state.pose.y) / dt;
			
			// wrap angle delta to [-pi, pi]
      		double dtheta = std::remainder(pose_stamped.pose.theta - prev_.state.pose.theta, 2 * M_PI);
      		double wz = dtheta / dt;
			
			//estimated velocities for naive state estimation
			Eigen::Vector3d vel(vx, vy, wz);

			//get position vector from pose_stamped where vector is pose
			Eigen::Vector3d pose = Eigen::Map<Eigen::Vector3d>(pose_stamped.pose, 3);

			//combine position vector and velocity vector for state vector, not state matrix
			Eigen::Matrix<double, 6, 1> xvector  = Eigen::Matrix<double, 6, 1>.setZero();
			xvector.block<3, 1>(0,0) = vel;
			xvector.block<3, 1>(3,0) = pose;
			
		

        } else {
            prev_state_ready_ = true;
        }

        prev_.state = state;
        prev_.header = pose_stamped.header;

        SendStateEstimate(state);
    }


        void process_update(double dt) {
            if (dt <= 0.) {
                return;
            }
	    /*A matrix used for prediction of next state matrix -- just for factoring in naive estimation into the 
     		next position state matrix
	    	Format:{ 
      			{ 1 0 0 d 0 0}
	 			{ 0 1 0 0 d 0}
    			{ 0 0 1 0 0 d}
       			{ 0 0 0 1 0 0}
				{ 0 0 0 0 1 0}
   				{ 0 0 0 0 0 1}
     			}
			where d = dt 
     		*/
            Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity(6, 6);
            A.block<3, 3>(0, 3).setIdentity() * dt;

			//[6 x 6] * [6 x 1] = [6 x 1]
            x = A * xvector;
			//
            P = A * P * A.transpose() + Q * dt;
        }

        void measurement_update(Eigen::VectorXd z) {
	    /*H matrix 
	    	Format:{ 
      			{ 1 0 0 0 0 0}
	 			{ 0 1 0 0 0 0}
    			{ 0 0 1 0 0 0}
     			}
			where d = dt 
     		*/
            Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero(3, 6);
            H.block<3, 3>(0, 0).setIdentity();

            Eigen::MatrixXd S = H * P * H.transpose() + R;
            Eigen::MatrixXd K = P * H.transpose() * S.inverse();
            Eigen::VectorXd y = z - H * x;
            y(2) = wrap_angle(y(2));

            x += K * y;
            P -= K * H * P;
        }

        double wrap_angle(double theta) {
            return atan2(sin(theta), cos(theta));
        }

    private:
        Eigen::VectorXd x;
        Eigen::MatrixXd P;
};


#include "ff_estimate/base_mocap_estimator.hpp"

class ConstVelKalmanFilterNode : public ff::BaseMocapEstimator {
    public:
        ConstVelKalmanFilterNode() : ff::BaseMocapEstimator("const_vel_kalman_filter_node") {
            this->declare_parameter("min_dt", 0.005);
            this->target_pose = Pose2D;
        }

    void EstimatewithPose2D(const Pose2DStamped & pose_stamped) override {
        FreeFlyerState state{};
        Pose2D pose2d{};

        state.pose = pose_stamped.pose;
        if (prev_state_ready_) {
            const rclcpp::Time now = pose_stamped.header.stamp;
            const rclcpp::Time last = prev_.header.stamp;
            double dt = (now - last).seconds();

            if (dt < (this->get_parameter("min_dt").as_double())) {
                return;
            }

            target_pose.pose.x = pose2d.pose.position.x;
            target_pose.pose.y = pose2d.pose.position.y;
            target_pose.pose.theta = pose2d.pose.position.theta;

            state.header = est_state.header
            state.state.twist = pose_stamped.state.twist;
            state.state.pose.x = this.target_pose.x;
            state.state.pose.y = this.target_pose.y;
            state.state.pose.theta = this.target_pose.theta;
        } else {
            prev_state_ready_ = true;
        }

        prev_.state = state;
        prev_.header = pose_stamped.header;

        SendStateEstimate(state);
    }

    private:
        geometry_msgs::msg::TwistStamped;
        geometry_msgs::msg::Pose2DStamped;
        ff_msgs::msg::FreeFlyerStateStamped prev_;
        bool prev_state_ready_ = false;
        geometry_msgs::msg::Pose2D;

        /* void target_loop() {
            if (!target_pose_.has_value()) {
                return;
            }

            auto target = std::make_shared<geometry_msgs::msg::TwistStamped>();
            target->header.stamp = now();
            target->twist.linear.x = target_pose_->x;
            target->twist.linear.y = target_pose_->y - 0.5;
            target->twist.angular.z = target_pose_->theta;

            target_pub_->publish(target);
        }

        void est_callback(const geometry_msgs::msg::Pose2DStamped::SharedPtr cv_pose) {
            if (!target_pose_.has_value()) {
                return;
            }

            auto state = std::make_shared<geometry_msgs::msg::TwistStamped>();
            state->header = cv_pose->header;
            state->twist = cv_pose->pose;
            state->twist.linear.x += target_pose_->x;
            state->twist.linear.y += target_pose_->y;
            state->twist.angular.z += target_pose_->theta;

            state_pub_->publish(state);
        }

        void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr target_pose) {
            if (!target_pose_.has_value()) {
                target_pose_ = geometry_msgs::msg::Pose2D();
            }

            target_pose_->x = target_pose->pose.position.x;
            target_pose_->y = target_pose->pose.position.y;
            target_pose_->theta = M_PI / 2.0;
        } */
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConstantVelKalmanFilterNode>());
    rclcpp::shutdown();
}
