#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

using namespace std::chrono_literals;

class KinematicsSolver : public rclcpp::Node
{
public:
  KinematicsSolver() : Node("kinematics_solver")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("end_effector_pose", 10);
    
    // Subscriber for real joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&KinematicsSolver::joint_state_callback, this, std::placeholders::_1));

    // Subscriber for target pose (IK)
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", 10, std::bind(&KinematicsSolver::target_pose_callback, this, std::placeholders::_1));

    // DH Parameters for 6-axis arm (simplified example)
    // a (link length), alpha (link twist), d (link offset), theta (joint angle)
    dh_params_ = {
      {0.0, -M_PI_2, 0.3, 0.0},
      {0.4, 0.0, 0.0, 0.0},
      {0.0, M_PI_2, 0.0, 0.0},
      {0.0, -M_PI_2, 0.3, 0.0},
      {0.0, M_PI_2, 0.0, 0.0},
      {0.0, 0.0, 0.1, 0.0}
    };
    
    current_joints_.resize(6, 0.0);
    
    RCLCPP_INFO(this->get_logger(), "Kinematics Solver Node Started. Waiting for /target_pose...");
  }

private:
  struct DHParam {
    double a;
    double alpha;
    double d;
    double theta;
  };

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < 6) return;

    // Update theta angles from joint state
    for (size_t i = 0; i < 6; ++i) {
      dh_params_[i].theta = msg->position[i];
      current_joints_[i] = msg->position[i];
    }

    // Calculate Forward Kinematics
    Eigen::Matrix4d T = compute_fk(current_joints_);

    // Publish Pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "base_link";

    pose_msg.pose.position.x = T(0, 3);
    pose_msg.pose.position.y = T(1, 3);
    pose_msg.pose.position.z = T(2, 3);

    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Quaterniond q(R);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    publisher_->publish(pose_msg);
  }

  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received target pose: (%.3f, %.3f, %.3f)", 
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    Eigen::Vector3d target_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    // For simplicity in this numerical IK, we'll just solve for Position (3 DOF target).
    // Full 6 DOF (Position + Orientation) requires more complex Jacobian and error calculations.
    
    std::vector<double> ik_joints = current_joints_;
    
    int max_iterations = 100;
    double tolerance = 1e-3;
    double learning_rate = 0.5;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
      Eigen::Matrix4d T = compute_fk(ik_joints);
      Eigen::Vector3d current_pos(T(0,3), T(1,3), T(2,3));
      
      Eigen::Vector3d error = target_pos - current_pos;
      if (error.norm() < tolerance) {
        RCLCPP_INFO(this->get_logger(), "IK Converged in %d iterations.", iter);
        print_joints(ik_joints);
        return; // Success
      }
      
      // Compute Jacobian (3x6 for position only)
      Eigen::Matrix<double, 3, 6> J = compute_jacobian_position(ik_joints);
      
      // Pseudo-inverse: J+ = J^T * (J * J^T)^-1
      Eigen::Matrix<double, 6, 3> J_pinv = J.transpose() * (J * J.transpose()).inverse();
      
      // Delta Theta
      Eigen::Matrix<double, 6, 1> dTheta = J_pinv * error;
      
      // Update joints
      for (int i = 0; i < 6; ++i) {
        ik_joints[i] += learning_rate * dTheta(i);
      }
    }
    
    RCLCPP_WARN(this->get_logger(), "IK did not converge!");
  }

  Eigen::Matrix4d compute_fk(const std::vector<double>& joints)
  {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < 6; ++i) {
      DHParam p = dh_params_[i];
      p.theta = joints[i];
      T = T * calculate_dh_transform(p);
    }
    return T;
  }

  Eigen::Matrix<double, 3, 6> compute_jacobian_position(std::vector<double>& joints)
  {
    Eigen::Matrix<double, 3, 6> J;
    double delta = 1e-5;
    
    Eigen::Matrix4d T_base = compute_fk(joints);
    Eigen::Vector3d pos_base(T_base(0,3), T_base(1,3), T_base(2,3));
    
    for (int i = 0; i < 6; ++i) {
      joints[i] += delta;
      Eigen::Matrix4d T_shifted = compute_fk(joints);
      Eigen::Vector3d pos_shifted(T_shifted(0,3), T_shifted(1,3), T_shifted(2,3));
      
      J.col(i) = (pos_shifted - pos_base) / delta;
      joints[i] -= delta; // restore
    }
    return J;
  }

  Eigen::Matrix4d calculate_dh_transform(const DHParam& p)
  {
    Eigen::Matrix4d T;
    double ct = std::cos(p.theta);
    double st = std::sin(p.theta);
    double ca = std::cos(p.alpha);
    double sa = std::sin(p.alpha);

    T << ct, -st * ca,  st * sa, p.a * ct,
         st,  ct * ca, -ct * sa, p.a * st,
         0,   sa,       ca,      p.d,
         0,   0,        0,       1;
    return T;
  }

  void print_joints(const std::vector<double>& joints) {
    std::string out = "IK Result Joints: [";
    for(int i=0; i<6; ++i) {
      out += std::to_string(joints[i]) + (i<5 ? ", " : "]");
    }
    RCLCPP_INFO(this->get_logger(), "%s", out.c_str());
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  
  std::vector<DHParam> dh_params_;
  std::vector<double> current_joints_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsSolver>());
  rclcpp::shutdown();
  return 0;
}
