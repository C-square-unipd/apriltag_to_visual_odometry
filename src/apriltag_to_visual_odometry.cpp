#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <chrono>
#include <string>
#include <cmath>
#include <algorithm>

#include <iostream>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <tf2/exceptions.h>

#include <eigen3/Eigen/Dense>

using std::placeholders::_1;
using namespace std::chrono;
using namespace std::chrono_literals;

enum OutliersFilterMethod
{
    None,
    JustTwo,
    Euclidean,
    Interquartiles,
    Mean,
    Median,
    Interquartiles2
};

enum FirMethod
{
    Disabled,
    Standard,
    EKF
};

enum LimitToSize
{
    Deactivated,
    One,
    Two
};

// Odometry publisher, "Node" subclass
class OdometryPublisher : public rclcpp::Node
{
public:
    // Constructor
    OdometryPublisher() : Node("apriltag_to_visual_odometry")
    {
        // Getting parameter from yaml file
        debug_ = declare_parameter<bool>("debug", false);
        graphics_on_ = declare_parameter<bool>("graphics_on", false);

        // CAMERA FRAME:
        fromFrameRel_ = declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");

        // CAMERA ROTATION:
        // PI_GRECO = 3.14159265 --> M_PI = 3.14159
        // PI_GRECO/2 = 1.5707963 --> M_PI_2 = 1.5708
        roll_cam_ = declare_parameter<int>("roll_cam", 0) * M_PI / 180.0;
        pitch_cam_ = declare_parameter<int>("pitch_cam", 0) * M_PI / 180.0;
        yaw_cam_ = declare_parameter<int>("yaw_cam", 0) * M_PI / 180.0;

        // CAMERA OFFSET:
        std::vector<double> offset_camera_body = declare_parameter<std::vector<double>>("offset_camera_body", {0.0, 0.0, 0.0});
        offset_camera_body_vect_ = {offset_camera_body[0], offset_camera_body[1], offset_camera_body[2]};
        std::vector<double> offset_body_camera = declare_parameter<std::vector<double>>("offset_body_camera", {0.0, 0.0, 0.0});
        offset_body_camera_vect_ = {offset_body_camera[0], offset_body_camera[1], offset_body_camera[2]};

        tag_ids_ = declare_parameter<std::vector<int64_t>>("tag_ids", std::vector<int64_t>{});

        // apriltag location vectors from the YAML file
        std::vector<double> tags_locations_XL = declare_parameter<std::vector<double>>("tags_locations_XL", {0.0, 0.0});
        std::vector<double> tags_locations_L = declare_parameter<std::vector<double>>("tags_locations_L", {0.0, 0.0});
        std::vector<double> tags_locations_M = declare_parameter<std::vector<double>>("tags_locations_M", {0.0, 0.0});
        std::vector<double> tags_locations_S = declare_parameter<std::vector<double>>("tags_locations_S", {0.0, 0.0});

        // averaging parameters
        use_chordal_avg_ = declare_parameter<bool>("chordal_averaging", false);
        frame_weight_ = declare_parameter<std::vector<int64_t>>("frame_weight", std::vector<int64_t>{1, 4, 16, 64});
        use_dynamic_weighting_ = declare_parameter<bool>("dynamic_weighting", false);

        // filtering parameters
        just_bigger_one_ = declare_parameter<bool>("just_bigger_one", false);
        limit_to_big_sizes = static_cast<LimitToSize>(declare_parameter<int>("limit_to_big_sizes", 0));
        filter_choice = static_cast<OutliersFilterMethod>(declare_parameter<int>("outliers_filter_choice", 6));
        euc_dist_max = declare_parameter<double>("euc_dist_max", 0.05);
        euc_outlier_ratio = declare_parameter<double>("euc_outlier_ratio", 0.2);
        euc_dist_to_increase = declare_parameter<double>("euc_dist_to_increase", 0.01);
        euc_use_ekf_ = declare_parameter<bool>("euc_use_ekf", true);
        fir_choice = static_cast<FirMethod>(declare_parameter<int>("fir", 1));
        fir_weight_ = declare_parameter<std::vector<int64_t>>("fir_weight", std::vector<int64_t>{1, 1, 1, 1, 1});

        if (filter_choice == OutliersFilterMethod::JustTwo)
            frame_weight_ = {1, 4, 16, 64}; // Forcing to use quadratic weights

        // Counter
        msgs_count_ = 0;
        msgs_count_old = 0;
        time_start_ = 0;
        time_count_ = 0;

        // To publish static transforms once at startup
        if (graphics_on_)
        {
            tf_static_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        }

        tf2::Vector3 no_translation = {0, 0, 0};
        tf2::Quaternion quat_x_180 = {-1, 0, 0, 0}; // 180 deg x-axis rotation
        this->make_static_transforms(home_rviz_frame_, home_map_frame_, no_translation, quat_x_180);

        if (debug_)
        {
            std::cout << "---------------------APRILTAG-MAP---------------------" << std::endl;
        }

        // Loading locations apriltag_map
        // From 4 vector<double> loaded form the YAML file to one vector<tf2> containing all the locations
        std::vector<tf2::Vector3> tags_locations;

        add_vectors_locations(tags_locations_XL_, tags_locations_XL);
        add_vectors_locations(tags_locations_L_, tags_locations_L);
        add_vectors_locations(tags_locations_M_, tags_locations_M);
        add_vectors_locations(tags_locations_S_, tags_locations_S);

        add_vectors_locations(tags_locations, tags_locations_XL);
        add_vectors_locations(tags_locations, tags_locations_L);
        add_vectors_locations(tags_locations, tags_locations_M);
        add_vectors_locations(tags_locations, tags_locations_S);

        // Size Check
        // tag_ids_ contains all the aprilag IDs, parameter declared on the YAML file
        if (tags_locations.size() != tag_ids_.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Error on declaration of the apriltag_map: SIZE INCORRECT!");
        }

        // Once at the startup print on the terminal tag location and publish all the frame static transforms
        std::string tag_frame;
        for (int i = 0; i < static_cast<int>(tag_ids_.size()); i++)
        {
            tag_frame = "frame";
            tag_frame.append(std::to_string(tag_ids_[i]));
            if (debug_)
            {
                std::cout << "tag_id = " << tag_ids_[i] << " --> " << tag_frame << " --> "
                          << "tag_location_ [ x: " << tags_locations[i][0] << ", y: " << tags_locations[i][1]
                          << ", z: " << tags_locations[i][2] << " ]" << std::endl; 
            }
            if (graphics_on_)
            {
                std::string child_frame = std::to_string(tag_ids_[i]);
                tf2::Vector3 translation = {tags_locations[i][0], tags_locations[i][1], tags_locations[i][2]};
                tf2::Quaternion quat_null = {0, 0, 0, 1}; // no rotation
                // Publish static transforms once at startup
                this->make_static_transforms(home_map_frame_, child_frame, translation, quat_null);
            }
        }
        if (debug_)
        {
            std::cout << "------------------------------------------------------" << std::endl
                      << std::endl;
            std::cout << "-------------------CAMERA-PARAMETER-------------------\n"
                      << "ANGLE SETTINGS:\n"
                      << "roll_cam = " << roll_cam_ << " [rad]\n"
                      << "pitch_cam = " << pitch_cam_ << " [rad]\n"
                      << "yaw_cam = " << yaw_cam_ << " [rad]\n"
                      << "offset_camera_body = "
                      << "[ x: " << offset_camera_body_vect_[0]
                      << ", y: " << offset_camera_body_vect_[1]
                      << ", z: " << offset_camera_body_vect_[2] << " ]  [m]\n"
                      << "camera_frame = " << fromFrameRel_ << std::endl
                      << "------------------------------------------------------\n"
                      << std::endl;

            std::cout << "------------------FILTERS-PARAMETER-------------------\n"
                      << "Consider only the bigger frame: " << just_bigger_one_ << std::endl
                      << "Outliers filter method: " << outliers_filter_names[static_cast<int>(filter_choice)] << std::endl
                      << "Limit to bigger sizes: " << size_limiter_names[static_cast<int>(limit_to_big_sizes)] << std::endl
                      << "Using chordal averaging: " << use_chordal_avg_ << std::endl
                      << "Using dynamic weighting: " << use_dynamic_weighting_ << std::endl
                      << "Final FIR: " << fir_methods_names[static_cast<int>(fir_choice)] << std::endl
                      << "------------------------------------------------------\n"
                      << std::endl;

            // to clear the log text file
            // std::ofstream log_stream(log_file_path, std::ios::out | std::ios::trunc);
            // log_stream << "";
        }
        // Definition of the elementary quaternion camera to body
        //  From (roll_angle, pitch_angle, yaw_angle) to quaternion
        quat_cam_to_body_x.setRPY(roll_cam_, 0, 0);
        quat_cam_to_body_y.setRPY(0, pitch_cam_, 0);
        quat_cam_to_body_z.setRPY(0, 0, yaw_cam_);

        if (graphics_on_)
        {
            // Initialize the transform broadcaster
            tf_drone_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            tf_ekf_drone_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // vehicle_odometry subscriber
            vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                "fmu/vehicle_odometry/out", 10, std::bind(&OdometryPublisher::vehicle_odometry_callback, this, _1));
        }

        // Create VehicleVisualOdometry publisher to PX4
        publisher_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("fmu/vehicle_visual_odometry/in", 10);

        // get common timestamp
        timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
                                                                           [this](const px4_msgs::msg::Timesync::UniquePtr msg)
                                                                           {
                                                                               timestamp_.store(msg->timestamp);
                                                                           });

        // Create subscription to tf
        subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(

            "/tf_vio", 10, std::bind(&OdometryPublisher::tf_callback, this, _1));

        // Definition of the elementary quaternion camera to body
        quat_cam_to_body_x.setRPY(roll_cam_, 0, 0);
        quat_cam_to_body_y.setRPY(0, pitch_cam_, 0);
        quat_cam_to_body_z.setRPY(0, 0, yaw_cam_);
        tf2::Quaternion quat_cam_to_body = quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;
        tf2::Quaternion quat_body_to_cam = quat_cam_to_body.inverse();

        // Publish the transformation considering camera offset and rotations relative to frame
        this->make_static_transforms(drone_frame_estimated_, fromFrameRel_, offset_body_camera_vect_, quat_body_to_cam);
    }

private:
    // convert location vector<double> to vector<tf2>
    void add_vectors_locations(std::vector<tf2::Vector3> &all_size_tags_locations, std::vector<double> &one_size_tags_locations)
    {
        for (int i = 0; i < static_cast<int>(one_size_tags_locations.size()); i = i + 2)
        {
            tf2::Vector3 location_to_insert = {one_size_tags_locations[i], one_size_tags_locations[i + 1], 0.0};
            all_size_tags_locations.push_back(location_to_insert);
        }
    }

    // make the transformation from the vector location and publish it (RVIZ)
    void make_static_transforms(std::string &parent_frame, std::string &child_frame, tf2::Vector3 &location, tf2::Quaternion &quat)
    {
        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped ts;

        ts.header.stamp = now;
        ts.header.frame_id = parent_frame;
        ts.child_frame_id = child_frame;

        ts.transform.translation.x = location[0];
        ts.transform.translation.y = location[1];
        ts.transform.translation.z = location[2];
        ts.transform.rotation.x = quat[0];
        ts.transform.rotation.y = quat[1];
        ts.transform.rotation.z = quat[2];
        ts.transform.rotation.w = quat[3];

        tf_static_publisher_->sendTransform(ts);
    }

    // Callback to publish the EKF transforms
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        rclcpp::Time now = this->get_clock()->now();

        tf_ekf_drone.header.stamp = now;
        tf_ekf_drone.header.frame_id = home_map_frame_;
        tf_ekf_drone.child_frame_id = "ekf_drone";

        tf_ekf_drone.transform.translation.x = msg->position[0];
        tf_ekf_drone.transform.translation.y = msg->position[1];
        tf_ekf_drone.transform.translation.z = msg->position[2];

        // VehicleOdometry msg has quaternion defined like: q{w, x, y, z} = {scalar, vect(3)}
        // tf2::Quaternion is defined like: q{x, y, z, w} = {vect(3), scalar}
        tf2::Quaternion quat{msg->q[1], msg->q[2], msg->q[3], msg->q[0]};

        tf_ekf_drone.transform.rotation.x = quat[0];
        tf_ekf_drone.transform.rotation.y = quat[1];
        tf_ekf_drone.transform.rotation.z = quat[2];
        tf_ekf_drone.transform.rotation.w = quat[3];

        // Send the transformation
        tf_ekf_drone_broadcaster_->sendTransform(tf_ekf_drone);

        if (fir_choice == FirMethod::EKF)
        {
            // push back in the old EFKs tuple
        ekf_transforms.push_back(std::make_tuple(
            tf_ekf_drone.transform.rotation.x,
            tf_ekf_drone.transform.rotation.y,
            tf_ekf_drone.transform.rotation.z,
            tf_ekf_drone.transform.rotation.w,
            tf_ekf_drone.transform.translation.x,
            tf_ekf_drone.transform.translation.y,
            tf_ekf_drone.transform.translation.z,
            1,
            0));

        // erase the oldest if the tuple is complete
        if (static_cast<int>(ekf_transforms.size()) >= static_cast<int>(fir_weight_.size()))
            ekf_transforms.erase(ekf_transforms.begin());
        }
    }

    // retrieve the frame weight given its ID
    int get_static_weight(int frame_id)
    {
        if (frame_id <= 99)
        {
            return frame_weight_[3];
        }
        else if (frame_id >= 100 && frame_id <= 399)
        {
            return frame_weight_[2];
        }
        else if (frame_id >= 400 && frame_id <= 999)
        {
            return frame_weight_[1];
        }
        else
        {
            return frame_weight_[0];
        }
    }

    int get_dynamic_weight(int frame_id, double x_coord, double y_coord, double z_coord)
    {
        float distance = sqrt(pow(x_coord, 2) + pow(y_coord, 2) + pow(z_coord, 2));
        if (frame_id <= 99)
        {
            return round((frame_weight_[3] / distance) * 100);
        }
        else if (frame_id >= 100 && frame_id <= 399)
        {
            return round((frame_weight_[2] / distance) * 100);
        }
        else if (frame_id >= 400 && frame_id <= 999)
        {
            return round((frame_weight_[1] / distance) * 100);
        }
        else
        {
            return round((frame_weight_[0] / distance) * 100);
        }
    }

    tf2::Vector3 get_offset(int frame_id)
    {
        tf2::Vector3 offset_home_to_apriltag;
        int location_index = 0;
        if (frame_id <= 99)
        {
            location_index = frame_id;
            offset_home_to_apriltag = {tags_locations_XL_[location_index][0], tags_locations_XL_[location_index][1], tags_locations_XL_[location_index][2]};
        }
        else if (frame_id >= 100 && frame_id <= 399)
        {
            location_index = frame_id - 100;
            offset_home_to_apriltag = {tags_locations_L_[location_index][0], tags_locations_L_[location_index][1], tags_locations_L_[location_index][2]};
        }
        else if (frame_id >= 400 && frame_id <= 999)
        {
            location_index = frame_id - 400;
            offset_home_to_apriltag = {tags_locations_M_[location_index][0], tags_locations_M_[location_index][1], tags_locations_M_[location_index][2]};
        }
        else
        {
            location_index = frame_id - 1000;
            offset_home_to_apriltag = {tags_locations_S_[location_index][0], tags_locations_S_[location_index][1], tags_locations_S_[location_index][2]};
        }

        return offset_home_to_apriltag;
    }

    tf2::Quaternion chordalAverage()
    {
        // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
        Eigen::Matrix4Xd A = quaternions.transpose() * weights.asDiagonal() * quaternions;
        double w_sum = weights.sum();

        // normalise with the sum of the weights
        A /= w_sum;

        // Calculate eigenvector and eigenvalues
        Eigen::EigenSolver<Eigen::MatrixXd> es(A);
        Eigen::VectorXd eigenValues = es.eigenvalues().real();
        Eigen::MatrixXd eigenVectors = es.eigenvectors().real();

        // find the eigen vector corresponding to the largest eigen value
        Eigen::Index largestEigenValueIndex;
        eigenValues.maxCoeff(&largestEigenValueIndex);

        tf2::Quaternion average(
            eigenVectors(0, largestEigenValueIndex),
            eigenVectors(1, largestEigenValueIndex),
            eigenVectors(2, largestEigenValueIndex),
            eigenVectors(3, largestEigenValueIndex));

        return average;
    }

    tf2::Quaternion quaternionAverage()
    {
        // calculate the weighted (direct) quaternion average
        Eigen::MatrixXd q_weighted = weights.asDiagonal() * quaternions;
        Eigen::VectorXd q_avg = q_weighted.colwise().sum();

        // normalize to get a unit quaternion
        q_avg.normalize();

        tf2::Quaternion average(q_avg(0), q_avg(1), q_avg(2), q_avg(3));
        return average;
    }

    template <int index>
    struct TupleLess
    {
        template <typename Tuple>
        bool operator()(const Tuple &left, const Tuple &right) const
        {
            return std::get<index>(left) < std::get<index>(right);
        }
    };

    tf2::Vector3 translationAverage()
    {
        int w_sum = weights.sum();
        Eigen::RowVectorXd average = (weights * positions) / w_sum;

        tf2::Vector3 avg(average(0), average(1), average(2));

        return avg;
    }

    // Functions to remove outliers
    void just_two_size(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> &t)
    {
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t_filtered_two;
        t_filtered_two.clear();
        int bigger_weight = get_static_weight(lower_child_id);
        int weight_limit = sqrt(bigger_weight); // WARNING: only with quadratic weight

        for (int i = 0; i < static_cast<int>(t.size()); i++)
        {
            if (std::get<7>(t[i]) >= weight_limit)
            {
                t_filtered_two.push_back(t[i]);
            }
        }
        t = t_filtered_two;
    }

    // NEW VERSION
    void interquartiles_filter2(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> &t)
    {
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t_filtered;
        int t_size = static_cast<int>(t.size());
        float tot_weight = 0;
        int index, index_min, index_max, w_sum;
        double min_margins[3] = {0, 0, 0};
        double max_margins[3] = {0, 0, 0};

        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> v_tuple[3] = {t, t, t};
        std::sort(v_tuple[0].begin(), v_tuple[0].end(), TupleLess<4>());
        std::sort(v_tuple[1].begin(), v_tuple[1].end(), TupleLess<5>());
        std::sort(v_tuple[2].begin(), v_tuple[2].end(), TupleLess<6>());

        // Calculate the sum of all_weight
        for (int i = 0; i < t_size; i++)
        {
            tot_weight += std::get<7>(v_tuple[0][i]);
        }

        // Compute the weighted interquartile ranges for X component
        w_sum = 0;
        index = 0;
        index_min = -1;
        index_max = -1;
        while (w_sum < (0.875 * tot_weight))
        {
            w_sum += std::get<7>(v_tuple[0][index]);
            if (index_min == -1 && w_sum >= (tot_weight / 8))
                index_min = index;
            else if (index_max == -1 && w_sum >= 0.875 * tot_weight)
                index_max = index;
            index++;
        }
        min_margins[0] = std::get<(4)>(v_tuple[0][index_min]);
        max_margins[0] = std::get<(4)>(v_tuple[0][index_max]);

        // Compute the weighted interquartile ranges for Y component
        w_sum = 0;
        index = 0;
        index_min = -1;
        index_max = -1;
        while (w_sum < (0.875 * tot_weight))
        {
            w_sum += std::get<7>(v_tuple[1][index]);
            if (index_min == -1 && w_sum >= (tot_weight / 8))
                index_min = index;
            else if (index_max == -1 && w_sum >= 0.875 * tot_weight)
                index_max = index;
            index++;
        }
        min_margins[1] = std::get<(5)>(v_tuple[1][index_min]);
        max_margins[1] = std::get<(5)>(v_tuple[1][index_max]);

        // Compute the weighted interquartile ranges for Z component
        w_sum = 0;
        index = 0;
        index_min = -1;
        index_max = -1;
        while (w_sum < (0.875 * tot_weight))
        {
            w_sum += std::get<7>(v_tuple[2][index]);
            if (index_min == -1 && w_sum >= (tot_weight / 8))
                index_min = index;
            else if (index_max == -1 && w_sum >= 0.875 * tot_weight)
                index_max = index;
            index++;
        }
        min_margins[2] = std::get<(6)>(v_tuple[2][index_min]);
        max_margins[2] = std::get<(6)>(v_tuple[2][index_max]);

        // Keep only trensformations that lie in the acceptable range for each component
        for (int i = 0; i < t_size; i++)
        {
            if (std::get<4>(t[i]) >= min_margins[0] && std::get<4>(t[i]) <= max_margins[0] && std::get<5>(t[i]) >= min_margins[1] && std::get<5>(t[i]) <= max_margins[1] && std::get<6>(t[i]) >= min_margins[2] && std::get<6>(t[i]) <= max_margins[2])
            {
                t_filtered.push_back(t[i]);
            }
        }

        if (static_cast<int>(t_filtered.size()) > 0)
            t = t_filtered;
    }

    // OLD VERSION
    void interquartiles_filter(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> &t)
    {
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t_filtered_med;
        int sum = 0;
        int all_weight = 0;
        int t_size = static_cast<int>(t.size());
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> v_tuple[3] = {t, t, t};

        std::sort(v_tuple[0].begin(), v_tuple[0].end(), TupleLess<4>());
        std::sort(v_tuple[1].begin(), v_tuple[1].end(), TupleLess<5>());
        std::sort(v_tuple[2].begin(), v_tuple[2].end(), TupleLess<6>());

        // Calculate the sum of all_weight
        for (int i = 0; i < t_size; i++)
        {
            all_weight += std::get<7>(v_tuple[0][i]);
        }

        // Compute the weighted median and eraser the outliers for x y z
        for (int index = 0; index < 3; index++)
        {
            sum = 0;
            int i = 0;
            int size = static_cast<int>(t.size());
            while (i != size)
            {
                sum += std::get<7>(v_tuple[index][i]);
                if ((sum < all_weight / 8) || (sum > all_weight * 7 / 8))
                {
                    v_tuple[index].erase(v_tuple[index].begin() + i);
                    size--;
                }
                else
                {
                    i++;
                }
            }
        }

        t_filtered_med.clear();
        bool zero_tuple = true;

        // Keep only frames that appears in all 3 tuples
        int frame_i = 0;
        for (int i = 0; i < static_cast<int>(v_tuple[0].size()); i++)
        {
            frame_i = std::get<8>(v_tuple[0][i]);

            for (int j = 0; j < static_cast<int>(v_tuple[1].size()); j++)
            {
                if (std::get<8>(v_tuple[1][j]) == frame_i)
                {
                    for (int k = 0; k < static_cast<int>(v_tuple[2].size()); k++)
                    {
                        if (std::get<8>(v_tuple[2][k]) == frame_i)
                        {
                            t_filtered_med.push_back(v_tuple[0][i]);
                            zero_tuple = false;
                            break;
                        }
                    }
                    break;
                }
            }
        }

        if (!zero_tuple)
        {
            t = t_filtered_med;
        }
    }

    void euclidean_distance(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> &t)
    {
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t_filtered_euc;
        int t_size = static_cast<int>(t.size());
        double dist_euc;
        bool find_outliers = true;
        double euc_dist_to_sum = 0;
        int euc_dist_count = 0;
        double old_trans[3];

        while (find_outliers)
        {
            if (euc_use_ekf_)
            {
                old_trans[0] = tf_ekf_drone.transform.translation.x;
                old_trans[1] = tf_ekf_drone.transform.translation.y;
                old_trans[2] = tf_ekf_drone.transform.translation.z;
            }
            else
            {
                old_trans[0] = trans_average.getX();
                old_trans[1] = trans_average.getY();
                old_trans[2] = trans_average.getZ();
            }

            t_filtered_euc.clear();
            for (int i = 0; i < t_size; i++)
            {
                dist_euc = sqrt(pow((std::get<4>(t[i]) - old_trans[0]), 2.0) +
                                pow((std::get<5>(t[i]) - old_trans[1]), 2.0) +
                                pow((std::get<6>(t[i]) - old_trans[2]), 2.0));

                if (dist_euc < euc_dist_max + euc_dist_to_sum)
                {
                    t_filtered_euc.push_back(t[i]);
                }
            }

            euc_dist_count++;

            if ((static_cast<double>(t_filtered_euc.size()) / t_size) < euc_outlier_ratio)
            {
                euc_dist_to_sum += euc_dist_to_increase;
            }
            else
            {
                find_outliers = false;
            }

            if (euc_dist_count > 30)
            {
                find_outliers = false;
                t_filtered_euc = t;
            }
        }

        t = t_filtered_euc;

        if (debug_)
        {
            std::cout << "Euclidean distance algorithm iteration: " << euc_dist_count << std::endl;
        }
    }

    Eigen::Vector3d standard_deviation(Eigen::MatrixXd positions, int size)
    {
        Eigen::Vector3d variances = Eigen::Vector3d::Zero();
        Eigen::Vector3d means = positions.colwise().mean();

        for (int i = 0; i < size; i++)
        {
            variances(0) += pow((positions(i, 0) - means(0)), 2);
            variances(1) += pow((positions(i, 1) - means(1)), 2);
            variances(2) += pow((positions(i, 2) - means(2)), 2);
        }
        variances /= (size - 1);

        return variances.cwiseSqrt();
    }

    Eigen::RowVector3d median(Eigen::MatrixXd positions, int size)
    {
        Eigen::Vector3d medians = Eigen::Vector3d::Zero();
        Eigen::VectorXd column;
        int middle = size / 2;
        for (int i = 0; i < 3; i++)
        {
            column = positions.col(i);
            std::sort(std::begin(column), std::end(column));
            if (size % 2)
                medians(i) = column(middle);
            else
                medians(i) = (column(middle) + column(middle - 1)) / 2;
        }
        return medians.transpose();
    }

    void mean_distance_filter(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> &t)
    {
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t_filtered;
        int t_size = static_cast<int>(t.size());
        const int d = 2; // parameter: the maximum distance allowed from the mean is d*standard_deviation
        double dist = 0;

        Eigen::MatrixXd positions(t_size, 3);
        for (int i = 0; i < t_size; i++)
        {
            positions(i, 0) = std::get<4>(t[i]);
            positions(i, 1) = std::get<5>(t[i]);
            positions(i, 2) = std::get<6>(t[i]);
        }
        Eigen::Vector3d barycenter = positions.colwise().mean(); // computed component-wise mean
        Eigen::Vector3d std_devs = standard_deviation(positions, t_size);
        double mean_std_dev = std_devs.mean(); // computed standard deviation of the data

        for (int i = 0; i < t_size; i++)
        {
            dist = sqrt(pow((positions(i, 0) - barycenter(0)), 2) + pow((positions(i, 1) - barycenter(1)), 2) + pow((positions(i, 2) - barycenter(2)), 2));
            if (dist <= d * mean_std_dev)
                t_filtered.push_back(t[i]);
        }

        if (static_cast<int>(t_filtered.size()) > 0)
            t = t_filtered;
    }

    void median_distance_filter(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> &t)
    {
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t_filtered;
        int t_size = static_cast<int>(t.size());
        const int d = 3;        // parameter: the maximum distance allowed from the median is d*SMAD
        const float c = 1.4826; // consistency constant (gaussian data distribution assumption)
        double dist = 0;

        Eigen::MatrixXd positions(t_size, 3);
        for (int i = 0; i < t_size; i++)
        {
            positions(i, 0) = std::get<4>(t[i]);
            positions(i, 1) = std::get<5>(t[i]);
            positions(i, 2) = std::get<6>(t[i]);
        }
        Eigen::RowVector3d medians = median(positions, t_size); // computed component-wise median
        Eigen::MatrixXd positions_MAD = positions.rowwise() - medians;
        Eigen::Vector3d compMADs = c * median(positions_MAD.cwiseAbs(), t_size);
        double sMAD = compMADs.mean(); // computed sMAD of the data

        for (int i = 0; i < t_size; i++)
        {
            dist = sqrt(pow((positions(i, 0) - medians(0)), 2) + pow((positions(i, 1) - medians(1)), 2) + pow((positions(i, 2) - medians(2)), 2));
            if (dist <= d * sMAD)
                t_filtered.push_back(t[i]);
        }

        if (static_cast<int>(t_filtered.size()) > 0)
            t = t_filtered;
    }

    void keep_bigger_size_filter(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> &t, bool take_two)
    {
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t_XL;
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t_L;
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t_M;
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t_S;
        int t_size = static_cast<int>(t.size());
        int frame_id;

        for (int i = 0; i < t_size; i++)
        {
            frame_id = std::get<8>(t[i]);
            if (frame_id <= 99)
                t_XL.push_back(t[i]);
            else if (frame_id >= 100 && frame_id <= 399)
                t_L.push_back(t[i]);
            else if (frame_id >= 400 && frame_id <= 999)
                t_M.push_back(t[i]);
            else
                t_S.push_back(t[i]);
        }

        if (static_cast<int>(t_XL.size()) > 0)
        {
            t = t_XL;
            if (take_two)
            {
                t.reserve(t.size() + t_L.size());
                t.insert(t.end(), t_L.begin(), t_L.end());
            }
        }
        else if (static_cast<int>(t_L.size()) > 0)
        {
            t = t_L;
            if (take_two)
            {
                t.reserve(t.size() + t_M.size());
                t.insert(t.end(), t_M.begin(), t_M.end());
            }
        }
        else if (static_cast<int>(t_M.size()) > 0)
        {
            t = t_M;
            if (take_two)
            {
                t.reserve(t.size() + t_S.size());
                t.insert(t.end(), t_S.begin(), t_S.end());
            }
        }
        else
            t = t_S;
    }

    // moving average filter
    void fir()
    {
        transforms.push_back(std::make_tuple(quat_body.x(),
                                             quat_body.y(),
                                             quat_body.z(),
                                             quat_body.w(),
                                             body_origin.getX(),
                                             body_origin.getY(),
                                             body_origin.getZ(),
                                             0,
                                             0));

        if (transforms.size() == (fir_weight_.size() + 1))
        {
            transforms.erase(transforms.begin());
            for (int i = 0; i < static_cast<int>(fir_weight_.size()); i++)
            {
                std::get<7>(transforms[i]) = fir_weight_[i];
            }
            process_transformations(transforms, false);
            body_origin = translationAverage();
            if (use_chordal_avg_)
                quat_body = chordalAverage();
            else
                quat_body = quaternionAverage();
        }
    }

    // moving average filter using transformations coming from EKF
    void ekf_fir()
    {
        if (ekf_transforms.size() == (fir_weight_.size()) - 1)
        {
            ekf_transforms.push_back(std::make_tuple(quat_body.x(),
                                                     quat_body.y(),
                                                     quat_body.z(),
                                                     quat_body.w(),
                                                     body_origin.getX(),
                                                     body_origin.getY(),
                                                     body_origin.getZ(),
                                                     0,
                                                     0));
            for (int i = 0; i < static_cast<int>(fir_weight_.size()); i++)
            {
                std::get<7>(ekf_transforms[i]) = fir_weight_[i];
            }
            process_transformations(ekf_transforms, true);
            body_origin = translationAverage();
            if (use_chordal_avg_)
                quat_body = chordalAverage();
            else
                quat_body = quaternionAverage();
            ekf_transforms.pop_back();
        }
    }

    void process_transformations(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t, bool quat_correction)
    {
        int t_size = static_cast<int>(t.size());
        Eigen::RowVector4d q_i = Eigen::RowVector4d::Zero();
        Eigen::RowVector4d ref_q{std::get<0>(t[0]), std::get<1>(t[0]), std::get<2>(t[0]), std::get<3>(t[0])};
        // Eigen::RowVector4d ref_q {tf_ekf_drone.transform.rotation.x, tf_ekf_drone.transform.rotation.y, tf_ekf_drone.transform.rotation.z, tf_ekf_drone.transform.rotation.w};
        double pos_q_distance = 0;
        double neg_q_distance = 0;

        quaternions.resize(t_size, 4);
        positions.resize(t_size, 3);
        weights.resize(t_size);

        for (int i = 0; i < t_size; i++)
        {
            quaternions(i, 0) = std::get<0>(t[i]);
            quaternions(i, 1) = std::get<1>(t[i]);
            quaternions(i, 2) = std::get<2>(t[i]);
            quaternions(i, 3) = std::get<3>(t[i]);

            if (quat_correction) // correct quaternion mismatching signs (due to double coverage)
            {
                q_i = quaternions.row(i);
                pos_q_distance = (ref_q - q_i).norm();
                neg_q_distance = (ref_q + q_i).norm();
                if (pos_q_distance > neg_q_distance)
                    quaternions.row(i) = -q_i; // correct the sign
            }

            positions(i, 0) = std::get<4>(t[i]);
            positions(i, 1) = std::get<5>(t[i]);
            positions(i, 2) = std::get<6>(t[i]);
            weights[i] = std::get<7>(t[i]);
        }
    }

    void print_transformations(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t)
    {
        int t_size = static_cast<int>(t.size());
        for (int i = 0; i < t_size; i++)
            std::cout << "\tq_x: " << std::get<0>(t[i]) << " "
                      << "q_y: " << std::get<1>(t[i]) << " "
                      << "q_z: " << std::get<2>(t[i]) << " "
                      << "q_w: " << std::get<3>(t[i]) << " "
                      << "x: " << std::get<4>(t[i]) << " "
                      << "y: " << std::get<5>(t[i]) << " "
                      << "z: " << std::get<6>(t[i]) << " "
                      << "w: " << std::get<7>(t[i]) << " "
                      << "frame: " << std::get<8>(t[i]) << std::endl;

        // log into a file if #poses > 3
        /*if (t_size > 3)
        //{
            std::ofstream log_stream;
            log_stream.open(log_file_path, std::ios::out | std::ios::app);
            for (int i = 0; i < t_size; i++)
            {
                log_stream << std::get<0>(t[i]) << " "
                           << std::get<1>(t[i]) << " "
                           << std::get<2>(t[i]) << " "
                           << std::get<3>(t[i]) << " "
                           << std::get<7>(t[i]) << " "
                           << std::get<4>(t[i]) << " "
                           << std::get<5>(t[i]) << " "
                           << std::get<6>(t[i]) << " "
                           << std::endl;
            }
            log_stream << std::endl;
            log_stream.close();
        //}*/
    }

    void tf_callback(tf2_msgs::msg::TFMessage::ConstSharedPtr tf_msg)
    {
        rclcpp::Time startAlgo = this->now();
        rclcpp::Time stop;
        rclcpp::Time start;
        const tf2_msgs::msg::TFMessage &msg_in = *tf_msg;

        if (msg_in.transforms.size() != 0)
        {
            geometry_msgs::msg::TransformStamped t_lower_id = msg_in.transforms[0];
            lower_child_id = std::stoi(t_lower_id.child_frame_id.substr(5, 4));

            // If the first useful transform arrived
            if (time_start_ == 0 || t_lower_id.header.stamp.sec == 0)
            {
                // update time_start_
                time_start_ = startAlgo.seconds();
                time_count_ = startAlgo.seconds();

                // Set the first transform based on the bigger apriltag seen
                trans_average[0] = (t_lower_id.transform.translation.x + get_offset(lower_child_id)[0]);
                trans_average[1] = (t_lower_id.transform.translation.y + get_offset(lower_child_id)[1]);
                trans_average[2] = (t_lower_id.transform.translation.z + get_offset(lower_child_id)[2]);
            }
            else
            {
                // Considering just the bigger frame, the one with the lower ID
                if (just_bigger_one_)
                {
                    int lower_frame_ID = std::stoi(t_lower_id.child_frame_id.substr(5, 4));

                    // checking if the marker ID is registered inside the tags map
                    if (!std::binary_search(tag_ids_.begin(), tag_ids_.end(), lower_frame_ID))
                    {
                        // no available tranformations left
                        std::cout << "** Warning: invalid transformation data from tf_vio topic, message discarded.";
                        return;
                    }

                    camera_origin[0] = t_lower_id.transform.translation.x;
                    camera_origin[1] = t_lower_id.transform.translation.y;
                    camera_origin[2] = t_lower_id.transform.translation.z;

                    quat_cam[0] = t_lower_id.transform.rotation.x;
                    quat_cam[1] = t_lower_id.transform.rotation.y;
                    quat_cam[2] = t_lower_id.transform.rotation.z;
                    quat_cam[3] = t_lower_id.transform.rotation.w;

                    quat_body = quat_cam * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;

                    body_origin = quatRotate(quat_cam, offset_camera_body_vect_) + camera_origin + get_offset(lower_frame_ID);
                }

                // Considering all the transforms received
                else
                {
                    // vector of tuple: (quat_x quat_y quat_z quat_w x y z weight frame_id)
                    std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> tuple_in;
                    int weight = 0;

                    // for every transform in the message received
                    for (size_t i = 0u; i < msg_in.transforms.size(); i++)
                    {
                        geometry_msgs::msg::TransformStamped ts_in = msg_in.transforms[i];
                        int child_id_num = std::stoi(ts_in.child_frame_id.substr(5, 4));

                        // checking if the marker ID is registered inside the tags map
                        if (!std::binary_search(tag_ids_.begin(), tag_ids_.end(), child_id_num))
                            continue;

                        tf2::Vector3 off_i = get_offset(child_id_num);

                        if (use_dynamic_weighting_)
                            weight = get_dynamic_weight(child_id_num, ts_in.transform.translation.x, ts_in.transform.translation.y, ts_in.transform.translation.z);
                        else
                            weight = get_static_weight(child_id_num);

                        // push back in the tuple vector
                        tuple_in.push_back(std::make_tuple(
                            ts_in.transform.rotation.x,
                            ts_in.transform.rotation.y,
                            ts_in.transform.rotation.z,
                            ts_in.transform.rotation.w,
                            ts_in.transform.translation.x + off_i[0],
                            ts_in.transform.translation.y + off_i[1],
                            ts_in.transform.translation.z + off_i[2],
                            weight,
                            child_id_num));
                    }
                    t_size = static_cast<int>(tuple_in.size());

                    if (t_size == 0)
                    {
                        // no available tranformations left
                        std::cout << "** Warning: invalid transformation data from tf_vio topic, message discarded.";
                        return;
                    }

                    // Print the received transforms
                    if (debug_)
                    {
                        std::cout << "-------------------------------------------------" << std::endl
                                  << "Trasformazioni ricevute: \n";
                        print_transformations(tuple_in);
                    }

                    // auto start = high_resolution_clock::now();
                    start = this->now();
                    if (limit_to_big_sizes == LimitToSize::One)
                        keep_bigger_size_filter(tuple_in, false);
                    else if (limit_to_big_sizes == LimitToSize::Two)
                    {
                        keep_bigger_size_filter(tuple_in, true);
                    }
                    
                    if (t_size > 2) // filter outliers only if there are at least 3 tranformations
                    {
                        switch (filter_choice)
                        {
                        case OutliersFilterMethod::None:
                            break;
                        case OutliersFilterMethod::JustTwo: // Consider the two bigger size of frames
                            just_two_size(tuple_in);
                            break;
                        case OutliersFilterMethod::Interquartiles: // Filter by considering weighted intequartiles ranges
                            interquartiles_filter(tuple_in);
                            break;
                        case OutliersFilterMethod::Euclidean: // Erase the outliers considering the Euclidian distance
                            euclidean_distance(tuple_in);
                            break;
                        case OutliersFilterMethod::Mean: // Erase the outliers far from the mean
                            mean_distance_filter(tuple_in);
                            break;
                        case OutliersFilterMethod::Median: // Erase the outliers far from the median
                            median_distance_filter(tuple_in);
                            break;
                        case OutliersFilterMethod::Interquartiles2:
                            interquartiles_filter2(tuple_in);
                            break;
                        default:
                            std::cout << "ERROR: invalid outliers filter selection, restoring default choice (Interquartiles)." << std::endl;
                            interquartiles_filter(tuple_in);
                            break;
                        }
                    }
                    stop = this->now();
                    // auto stop = high_resolution_clock::now();
                    // auto duration = duration_cast<microseconds>(stop - start);
                    t_f_size = static_cast<int>(tuple_in.size());

                    if (debug_)
                    {
                        std::cout << "-------------------------------------------------" << std::endl
                                  << "Trasformazioni filtrate: \n";
                        print_transformations(tuple_in);
                    }

                    // auto start = high_resolution_clock::now();
                    if (t_f_size > 1)
                    {
                        // Convert tuple to matrices and correct the quaternion signs, also correct quaternion signs if Quaternion Averaging is selected
                        process_transformations(tuple_in, !use_chordal_avg_);
                        // Average quaternions
                        if (use_chordal_avg_)
                            quat_average = chordalAverage();
                        else
                            quat_average = quaternionAverage();
                        // Average translations
                        trans_average = translationAverage();
                    }
                    else
                    {
                        quat_average = {std::get<0>(tuple_in[0]), std::get<1>(tuple_in[0]), std::get<2>(tuple_in[0]), std::get<3>(tuple_in[0])};
                        trans_average = {std::get<4>(tuple_in[0]), std::get<5>(tuple_in[0]), std::get<6>(tuple_in[0])};
                    }

                    // auto stop = high_resolution_clock::now();
                    // auto duration = duration_cast<microseconds>(stop - start);

                    // print algorithm execution time to file
                    /*if (debug_)
                    {
                        std::ofstream log_stream;
                        log_stream.open(log_file_path, std::ios::out | std::ios::app);
                        //log_stream << duration.count() << " " << t_f_size << std::endl;
                        log_stream << t_size << " " << t_f_size << std::endl;
                        log_stream.close();
                    }*/

                    // Final transforms
                    quat_body = quat_average * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;
                    body_origin = quatRotate(quat_average, offset_camera_body_vect_) + trans_average;
                }

                // FIR
                if (fir_choice == FirMethod::Standard)
                    fir();
                else if (fir_choice == FirMethod::EKF)
                    ekf_fir();

                // Generate the message
                msg.quality = 0;

                rclcpp::Time now = this->now();
                // msg.timestamp = now.nanoseconds() / 1000;
                long vio_arrival_time = static_cast<long>(t_lower_id.header.stamp.sec) * 1000000 + static_cast<long>(t_lower_id.header.stamp.nanosec) / 1000;
                long elapsed_time = (static_cast<long>(now.nanoseconds()) - static_cast<long>(startAlgo.nanoseconds())) / 1000;
                msg.timestamp = vio_arrival_time + elapsed_time;
                // msg.timestamp_sample = startAlgo.nanoseconds() / 1000;
                msg.timestamp_sample = vio_arrival_time;

                /*if (debug_)
                {
                    elapsed_time = (static_cast<long>(stop.nanoseconds()) - static_cast<long>(start.nanoseconds())) / 1000;
                    std::ofstream log_stream;
                    log_stream.open(log_file_path, std::ios::out | std::ios::app);
                    log_stream << elapsed_time << std::endl;
                    log_stream.close();
                }*/

                msg.pose_frame = 1;
                msg.position.at(0) = body_origin.getX();
                msg.position.at(1) = body_origin.getY();
                msg.position.at(2) = body_origin.getZ();
                msg.position_variance.at(0) = NAN;
                // msg.position_variance.at(1) = NAN;
                // msg.position_variance.at(2) = NAN;

                // VehicleVisualOdometry msg has quaternion defined like: q{w, x, y, z} = {scalar, vect(3)}
                // tf2::Quaternion is defined like: q{x, y, z, w} = {vect(3), scalar}
                msg.q.at(0) = quat_body[3];
                msg.q.at(1) = quat_body[0];
                msg.q.at(2) = quat_body[1];
                msg.q.at(3) = quat_body[2];
                msg.orientation_variance.at(0) = NAN;
                // msg.orientation_variance.at(1) = NAN;
                // msg.orientation_variance.at(2) = NAN;

                msg.velocity_frame = 0;
                msg.velocity.at(0) = NAN;
                // msg.velocity.at(1) = NAN;
                // msg.velocity.at(2) = NAN;
                msg.angular_velocity.at(0) = NAN;
                // msg.angular_velocity.at(1) = NAN;
                // msg.angular_velocity.at(2) = NAN;
                msg.velocity_variance.at(0) = NAN;
                // msg.velocity_variance.at(1) = NAN;
                // msg.velocity_variance.at(2) = NAN;

                if (debug_)
                {
                    std::cout << "Final pose:\n"
                              << "\t translations:\t[ x: " << msg.position.at(0) << ", y: " << msg.position.at(1) << ", z: " << msg.position.at(2) << " ]\n"
                              << "\t quaternion:\t[ w: " << msg.q[0] << ", ( x: " << msg.q[1] << ", y: " << msg.q[2] << ", z: " << msg.q[3] << ") ]\n"
                              << "-------------------------------------------------\n\n";
                    /*std::ofstream log_stream;
                    log_stream.open(log_file_path, std::ios::out | std::ios::app);
                    log_stream << msg.position.at(0) << " " << msg.position.at(1) << " " << msg.position.at(2) << std::endl;
                    log_stream.close();*/
                }
                if (graphics_on_)
                {
                    rclcpp::Time now = this->get_clock()->now();
                    geometry_msgs::msg::TransformStamped tf_drone;

                    tf_drone.header.stamp = now;
                    tf_drone.header.frame_id = home_map_frame_;
                    tf_drone.child_frame_id = drone_frame_estimated_;

                    tf_drone.transform.translation.x = msg.position.at(0);
                    tf_drone.transform.translation.y = msg.position.at(1);
                    tf_drone.transform.translation.z = msg.position.at(2);

                    // VehicleOdometry msg has quaternion defined like: q{w, x, y, z} = {scalar, vect(3)}
                    // tf2::Quaternion is defined like: q{x, y, z, w} = {vect(3), scalar}
                    tf2::Quaternion quat{msg.q[1], msg.q[2], msg.q[3], msg.q[0]};
                    tf_drone.transform.rotation.x = quat.getX();
                    tf_drone.transform.rotation.y = quat.getY();
                    tf_drone.transform.rotation.z = quat.getZ();
                    tf_drone.transform.rotation.w = quat.getW();

                    // Send the transformation
                    tf_drone_broadcaster_->sendTransform(tf_drone);
                }

                // Publish the VehicleVisualOdometry
                publisher_->publish(msg);

                // Print stats every five seconds
                msgs_count_++;
                if (startAlgo.seconds() - time_count_ > 5)
                {
                    std::cout << "Messages received:\t" << msgs_count_ << std::endl
                              << "Messages frequency:\t" << (msgs_count_ - msgs_count_old) / (startAlgo.seconds() - time_count_)
                              << " Hz" << std::endl
                              << "Pose:\n"
                              << "\t translations:\t[ x: " << msg.position.at(0) << ", y: " << msg.position.at(1) << ", z: " << msg.position.at(2) << " ]\n"
                              << "\t quaternion:\t[ w: " << msg.q[0] << ", ( x: " << msg.q[1] << ", y: " << msg.q[2] << ", z: " << msg.q[3] << ") ]\n"
                              << "----------------------------------------------------------------------------------------" << std::endl;
                    msgs_count_old = msgs_count_;
                    time_count_ = startAlgo.seconds();
                }
            }
        }
    }

    // Declare private variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;

    std::atomic<uint64_t> timestamp_; //!< common synced timestamped with PX4

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_drone_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_ekf_drone_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_publisher_;

    geometry_msgs::msg::TransformStamped tf_ekf_drone;
    std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> ekf_transforms;
    std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> transforms;

    // For a better visualization create two home one with z_up (home_rviz) and one with z_down (home_map)
    std::string home_map_frame_ = "home_map";   // z_down
    std::string home_rviz_frame_ = "home_rviz"; // z_up
    std::string drone_frame_estimated_ = "drone";

    // Parameter from yaml file
    bool debug_, graphics_on_, just_bigger_one_, euc_use_ekf_, use_chordal_avg_, use_dynamic_weighting_;
    OutliersFilterMethod filter_choice;
    FirMethod fir_choice;
    LimitToSize limit_to_big_sizes;
    std::string fromFrameRel_;
    float roll_cam_, pitch_cam_, yaw_cam_;                           //[deg]
    tf2::Vector3 offset_camera_body_vect_, offset_body_camera_vect_; //[m]
    std::vector<int64_t> tag_ids_, frame_weight_, fir_weight_;
    
    std::vector<tf2::Vector3> tags_locations_XL_;
    std::vector<tf2::Vector3> tags_locations_L_;
    std::vector<tf2::Vector3> tags_locations_M_;
    std::vector<tf2::Vector3> tags_locations_S_;

    int lower_child_id;
    int msgs_count_, msgs_count_old;
    int time_start_, time_count_;
    int t_size, t_f_size; // keep in memory the number of transformation before and after outliers filtering

    px4_msgs::msg::VehicleVisualOdometry msg;

    tf2::Quaternion quat_cam_to_body_x, quat_cam_to_body_y, quat_cam_to_body_z;
    tf2::Quaternion quat_body; // from apriltag-frame to body-frame
    tf2::Quaternion quat_cam;  // from apriltag-frame to camera-frame
    tf2::Quaternion quat_average;
    tf2::Vector3 camera_origin, body_origin, trans_average;
    Eigen::MatrixXd positions;
    Eigen::MatrixXd quaternions;
    Eigen::RowVectorXd weights;
    double euc_dist_max, euc_outlier_ratio, euc_dist_to_increase;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("apriltag_to_visual_odometry");
    // std::string log_file_path = package_share_directory + "/../../../../log/execution_time.txt";
    //  std::string log_file_path = "//media//simone//8GBGREEN//bags//tags_log";
    std::vector<std::string> outliers_filter_names = {"None", "Two bigger size", "Euclidean distance", "Interquartiles", "Distance from Mean", "distance from Median", "New interquartiles"};
    std::vector<std::string> fir_methods_names = {"Disabled", "Standard", "EKF"};
    std::vector<std::string> size_limiter_names = {"Disabled", "One", "Two"};
};

int main(int argc, char *argv[])
{
    std::cout << "Starting apriltag_to_visual_odometry node..." << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
