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

#include <geometry_msgs/msg/twist.hpp>
#include <tf2/exceptions.h>

#include <eigen3/Eigen/Dense>

using std::placeholders::_1;
using namespace std::chrono;
using namespace std::chrono_literals;


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

        // weight to apply from YAML file
        frame_weight_ = declare_parameter<std::vector<int64_t>>("frame_weight", std::vector<int64_t>{1, 4, 16, 64});


        // filtering parameters
        just_bigger_one_ = declare_parameter<bool>("just_bigger_one", false);
        euc_dist_filter_ = declare_parameter<bool>("euc_dist_filter", false);
        iqr_filter_ = declare_parameter<bool>("iqr_filter", true);
        iqr_boundaries = declare_parameter<double>("iqr_boundaries", 0.125);
        euc_dist_max = declare_parameter<double>("euc_dist_max", 0.05);
        euc_outlier_ratio = declare_parameter<double>("euc_outlier_ratio", 0.2);
        euc_dist_to_increase = declare_parameter<double>("euc_dist_to_increase", 0.01);
        just_two_size_ = declare_parameter<bool>("just_two_size", false);
        euc_use_ekf_ = declare_parameter<bool>("euc_use_ekf", true);
        fir_ = declare_parameter<bool>("fir", true);
        fir_weight_ = declare_parameter<std::vector<int64_t>>("fir_weight", std::vector<int64_t>{1, 1, 1, 1});

        // Counter
        msgs_count_ = 0;
        msgs_count_old = 0;
        time_start_ = 0;
        time_count_= 0;

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
        }

                std::cout   << "-------------------CAMERA-PARAMETER-------------------\n"
                    << "ANGLE SETTINGS:\n"
                    << "roll_cam = " << roll_cam_ << " [rad]\n"
                    << "pitch_cam = " << pitch_cam_ << " [rad]\n"
                    << "yaw_cam = " << yaw_cam_ << " [rad]\n"
                    << "offset_camera_body = "
                    << "[ x: " << offset_camera_body_vect_[0] 
                    << ", y: " << offset_camera_body_vect_[1] 
                    << ", z: " << offset_camera_body_vect_[2] << " ]  [m]\n"
                    << "camera_frame = " << fromFrameRel_ << std::endl
                    << "------------------------------------------------------\n" << std::endl;

                 std::cout   << "------------------FILTERS-PARAMETER-------------------\n"
                    << "Consider only the bigger frame: " << just_bigger_one_ << std::endl
                    << "Consider only the two bigger sizes of frame: " << just_two_size_ << std::endl
                    << "Filtering using euclidean distance: " << euc_dist_filter_ << std::endl
                    << "Filtering using weighted median: " << iqr_filter_ << std::endl
                    << "Threshold using weighted median: " << iqr_boundaries << std::endl
                    << "Final FIR: " << fir_ << std::endl
                    << "------------------------------------------------------\n" << std::endl;

                

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
    }

    int get_weight(int frame_id)
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

    tf2::Quaternion quaternionAverage(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> &t)
    {

        // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
        Eigen::Matrix4f A = Eigen::Matrix4f::Zero();
        Eigen::Vector4f q_i = Eigen::Vector4f::Zero();
        int w_i = 0;
        int w_sum = 0;
        int t_size = static_cast<int>(t.size());

        for (int i = 0; i < t_size; i++)
        {
            q_i(0) = std::get<0>(t[i]);
            q_i(1) = std::get<1>(t[i]);
            q_i(2) = std::get<2>(t[i]);
            q_i(3) = std::get<3>(t[i]);
            w_i = std::get<7>(t[i]);
            A += w_i * q_i * q_i.transpose();
            w_sum += w_i;
        }

        // normalise with the sum of the weights
        A /= w_sum;

        // Calculate eigenvector and eigenvalues
        Eigen::EigenSolver<Eigen::Matrix4f> es(A);
        Eigen::VectorXf eigenValues = es.eigenvalues().real();
        Eigen::MatrixXf eigenVectors = es.eigenvectors().real();

        // find the eigen vector corresponding to the largest eigen value
        int largestEigenValueIndex = 0;
        float largestEigenValue = eigenValues(0);

        for (int i = 1; i < eigenValues.rows(); ++i)
        {
            if (eigenValues(i) > largestEigenValue)
            {
                largestEigenValue = eigenValues(i);
                largestEigenValueIndex = i;
            }
        }

        tf2::Quaternion average(
            eigenVectors(0, largestEigenValueIndex),
            eigenVectors(1, largestEigenValueIndex),
            eigenVectors(2, largestEigenValueIndex),
            eigenVectors(3, largestEigenValueIndex));

        return average;
    }

    template<int index> struct TupleLess
    {
        template<typename Tuple>
        bool operator() (const Tuple& left, const Tuple& right) const
        {
            return std::get<index>(left) < std::get<index>(right);
        }
    };

    tf2::Vector3 translationAverage(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> &t)
    {
        double w_i = 0;
        double w_sum = 0;
        int t_size = static_cast<int>(t.size());
        tf2::Vector3 v_i(0, 0, 0);
        tf2::Vector3 v_average(0, 0, 0);

        for (int i = 0; i < t_size; i++)
        {
            w_i = std::get<7>(t[i]);
            v_i[0] = w_i * std::get<4>(t[i]);
            v_i[1] = w_i * std::get<5>(t[i]);
            v_i[2] = w_i * std::get<6>(t[i]);
            v_average += v_i;
            w_sum += w_i;
        }

        v_average[0] /= w_sum;
        v_average[1] /= w_sum;
        v_average[2] /= w_sum;

        return v_average;
    }

    // Functions to remove outliers
    void just_two_size(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> &t)
    {
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t_filtered_two;
        t_filtered_two.clear();
        int bigger_weight = get_weight(lower_child_id);
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

    void interquartile_range(std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> &t)
    {
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> t_filtered_med;
        int sum = 0;
        int all_weight = 0;
        int N = static_cast<int>(t.size());
        std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> v_tuple[3] = {t, t, t};

        std::sort(v_tuple[0].begin(), v_tuple[0].end(),  TupleLess<4>());
        std::sort(v_tuple[1].begin(), v_tuple[1].end(),  TupleLess<5>());                        
        std::sort(v_tuple[2].begin(), v_tuple[2].end(),  TupleLess<6>());
        
        // Calculate the sum of all_weight
        for (int i = 0; i < N; i++)
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
                if ((sum < all_weight * iqr_boundaries) || (sum > all_weight * (1-iqr_boundaries)))
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

    // moving average filter
    void fir()
    {
        transforms.push_back( std::make_tuple(  quat_body.x(),
                                                quat_body.y(),
                                                quat_body.z(),
                                                quat_body.w(),
                                                body_origin.getX(),
                                                body_origin.getY(),
                                                body_origin.getZ(),
                                                0,
                                                0 ));

        if (transforms.size() == (fir_weight_.size()+1))
        {
            transforms.erase(transforms.begin());
            for (int i=0; i < static_cast<int>(fir_weight_.size()); i++)
            {
                std::get<7>(transforms[i]) = fir_weight_[i];
            }
            quat_body = quaternionAverage(transforms);
            body_origin = translationAverage(transforms);
        }
    }

    void tf_callback(tf2_msgs::msg::TFMessage::ConstSharedPtr tf_msg)
    {
        rclcpp::Time startAlgo = this->now();

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

                    // for every transform in the message received
                    for (size_t i = 0u; i < msg_in.transforms.size(); i++)
                    {
                        geometry_msgs::msg::TransformStamped ts_in = msg_in.transforms[i];
                        int child_id_num = std::stoi(ts_in.child_frame_id.substr(5, 4));

                        tf2::Vector3 off_i = get_offset(child_id_num);

                        // push back in the tuple vector
                        tuple_in.push_back(std::make_tuple(
                            ts_in.transform.rotation.x,
                            ts_in.transform.rotation.y,
                            ts_in.transform.rotation.z,
                            ts_in.transform.rotation.w,
                            ts_in.transform.translation.x + off_i[0],
                            ts_in.transform.translation.y + off_i[1],
                            ts_in.transform.translation.z + off_i[2],
                            get_weight(child_id_num),
                            child_id_num));
                    }

                    // Print the received transforms
                    if (debug_)
                    {
                        std::cout << "-------------------------------------------------" << std::endl
                                  << "Trasformazioni ricevute: \n";
                        for (int i = 0; i < static_cast<int>(tuple_in.size()); i++)
                            std::cout << "\tq_x: " << std::get<0>(tuple_in[i]) << " "
                                      << "q_y: " << std::get<1>(tuple_in[i]) << " "
                                      << "q_z: " << std::get<2>(tuple_in[i]) << " "
                                      << "q_w: " << std::get<3>(tuple_in[i]) << " "
                                      << "x: " << std::get<4>(tuple_in[i]) << " "
                                      << "y: " << std::get<5>(tuple_in[i]) << " "
                                      << "z: " << std::get<6>(tuple_in[i]) << " "
                                      << "w: " << std::get<7>(tuple_in[i]) << " "
                                      << "frame: " << std::get<8>(tuple_in[i]) << std::endl;
                    }

                    // Consider the two bigger size of frames
                    if (just_two_size_)
                    {
                        just_two_size(tuple_in);
                    }

                    // Filtering by weighted median of the translations
                    if (iqr_filter_)
                    {
                        interquartile_range(tuple_in);
                    }

                    // Erase the outliers considering the Euclidian distance
                    if (euc_dist_filter_)
                    {
                        euclidean_distance(tuple_in);
                    }

                    if (debug_)
                    {
                        std::cout << "Trasformazioni filtrate: \n";
                        for (int i = 0; i < static_cast<int>(tuple_in.size()); i++)
                            std::cout << "\tq_x: " << std::get<0>(tuple_in[i]) << " "
                                      << "q_y: " << std::get<1>(tuple_in[i]) << " "
                                      << "q_z: " << std::get<2>(tuple_in[i]) << " "
                                      << "q_w: " << std::get<3>(tuple_in[i]) << " "
                                      << "x: " << std::get<4>(tuple_in[i]) << " "
                                      << "y: " << std::get<5>(tuple_in[i]) << " "
                                      << "z: " << std::get<6>(tuple_in[i]) << " "
                                      << "w: " << std::get<7>(tuple_in[i]) << " "
                                      << "frame: " << std::get<8>(tuple_in[i]) << std::endl;
                    }

                    // Average quaternions
                    quat_average = quaternionAverage(tuple_in);

                    // Average translations
                    trans_average = translationAverage(tuple_in);

                    // Final transforms
                    quat_body = quat_average * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;
                    body_origin = quatRotate(quat_average, offset_camera_body_vect_) + trans_average;
                }

                // FIR
                if(fir_)
                {
                    fir();
                }

                // Generate the message
                msg.quality = 0;
               
                rclcpp::Time now = this->now();
                msg.timestamp = now.nanoseconds() / 1000;
                msg.timestamp_sample = startAlgo.nanoseconds() / 1000;
                unsigned long sec_to_micro = (unsigned long) t_lower_id.header.stamp.sec;
                sec_to_micro = sec_to_micro * 1000000;
                unsigned long nano_to_micro = (unsigned long) t_lower_id.header.stamp.nanosec;
                nano_to_micro = nano_to_micro / 1000;
                unsigned long timeDetection = sec_to_micro + nano_to_micro;
                //msg.timestamp_sample = sec_to_micro + nano_to_micro;
                // msg.timestamp_sample = now.nanoseconds() / 1000;

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
                if(startAlgo.seconds() - time_count_ > 5)
                {
                    std::cout << "Messages received:\t" << msgs_count_ << std::endl
                              << "Messages frequency:\t" << (msgs_count_-msgs_count_old)/(startAlgo.seconds() - time_count_) 
                              << " Hz" << std::endl << "Pose:\n"
                              << "\t translations:\t[ x: " << msg.position.at(0) << ", y: " << msg.position.at(1) << ", z: " << msg.position.at(2) << " ]\n"
                              << "\t quaternion:\t[ w: " << msg.q[0] << ", ( x: " << msg.q[1] << ", y: " << msg.q[2] << ", z: " << msg.q[3] << ") ]\n"
                              << "----------------------------------------------------------------------------------------" << std::endl;
                    std::cout << "Actual time:\t" << now.nanoseconds() / 1000 << "\n"
                              << "Start algo t:\t"  << startAlgo.nanoseconds() / 1000 << "\n"
                              << "Detection t:\t"  <<  timeDetection << "\n"
                              << "Act - algo:" << now.nanoseconds() / 1000 - startAlgo.nanoseconds() / 1000 << "\n"
                              << "Act - det:" << now.nanoseconds() / 1000 - timeDetection << std::endl;
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
    std::vector<std::tuple<double, double, double, double, double, double, double, int, int>> transforms;

    // For a better visualization create two home one with z_up (home_rviz) and one with z_down (home_map)
    std::string home_map_frame_ = "home_map";   // z_down
    std::string home_rviz_frame_ = "home_rviz"; // z_up
    std::string drone_frame_estimated_ = "drone";

    // Parameter from yaml file
    bool debug_, graphics_on_, euc_dist_filter_, iqr_filter_, just_bigger_one_, just_two_size_, euc_use_ekf_, fir_;
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

    px4_msgs::msg::VehicleVisualOdometry msg;

    tf2::Quaternion quat_cam_to_body_x, quat_cam_to_body_y, quat_cam_to_body_z;
    tf2::Quaternion quat_body; // from apriltag-frame to body-frame
    tf2::Quaternion quat_cam;  // from apriltag-frame to camera-frame
    tf2::Quaternion quat_average;
    tf2::Vector3 camera_origin, body_origin, trans_average;
    double euc_dist_max, euc_outlier_ratio, euc_dist_to_increase, iqr_boundaries;
};

int main(int argc, char *argv[])
{
    std::cout << "Starting apriltag_to_visual_odometry node..." << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
