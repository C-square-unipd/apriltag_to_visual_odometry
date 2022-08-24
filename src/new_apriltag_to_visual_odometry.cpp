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

u_int64_t time_start = 0;
// u_int64_t time_start = 1;
u_int64_t delta_time = 0;

int old_number_of_frames = 0;

// Odometry publisher, "Node" subclass
class OdometryPublisher : public rclcpp::Node
{
public:
    // Constructor
    OdometryPublisher() : Node("apriltag_to_visual_odometry")
    {
        // Getting parameter from yaml file
        // declare_parameter<parameter_type>(parameter_name, default_value, parameter_descriptor)
        // return the effective value
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

        std::cout << "-------------------CAMERA-PARAMETER-------------------" << std::endl;
        std::cout << "ANGLE SETTINGS:" << std::endl;
        std::cout << "roll_cam = " << roll_cam_ << " [rad]" << std::endl;
        std::cout << "pitch_cam = " << pitch_cam_ << " [rad]" << std::endl;
        std::cout << "yaw_cam = " << yaw_cam_ << " [rad]" << std::endl;
        std::cout << "offset_camera_body = "
                  << "[ x: " << offset_camera_body_vect_[0] << ", y: " << offset_camera_body_vect_[1] << ", z: " << offset_camera_body_vect_[2] << " ]  [m]" << std::endl;
        std::cout << "camera_frame = " << fromFrameRel_ << std::endl;
        std::cout << "------------------------------------------------------" << std::endl
                  << std::endl;

        tag_ids_ = declare_parameter<std::vector<int64_t>>("tag_ids", std::vector<int64_t>{});

        // apriltag location vectors from the YAML file
        std::vector<double> tags_locations_XL = declare_parameter<std::vector<double>>("tags_locations_XL", {0.0, 0.0});
        std::vector<double> tags_locations_L = declare_parameter<std::vector<double>>("tags_locations_L", {0.0, 0.0});
        std::vector<double> tags_locations_M = declare_parameter<std::vector<double>>("tags_locations_M", {0.0, 0.0});
        std::vector<double> tags_locations_S = declare_parameter<std::vector<double>>("tags_locations_S", {0.0, 0.0});

        // weight to apply from YAML file
        // frame_weight = declare_parameter<std::vector<int>>("frame_weight", {1,4,16,64});
        frame_weight = {1, 4, 16, 64};

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
        geometry_msgs::msg::TransformStamped tf_ekf_drone;

        tf_ekf_drone.header.stamp = now;
        tf_ekf_drone.header.frame_id = home_map_frame_;
        tf_ekf_drone.child_frame_id = "ekf_drone";

        tf_ekf_drone.transform.translation.x = msg->x;
        tf_ekf_drone.transform.translation.y = msg->y;
        tf_ekf_drone.transform.translation.z = msg->z;

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
            return frame_weight[3];
        }
        else if (frame_id >= 100 && frame_id <= 399)
        {
            return frame_weight[2];
        }
        else if (frame_id >= 400 && frame_id <= 999)
        {
            return frame_weight[1];
        }
        else
        {
            return frame_weight[0];
        }
    }

    void print_quaternion(geometry_msgs::msg::TransformStamped ts)
    {
        tf2::Quaternion q(
            ts.transform.rotation.x,
            ts.transform.rotation.y,
            ts.transform.rotation.z,
            ts.transform.rotation.w);

        std::cout << "x: " << ts.transform.rotation.x
                  << " y: " << ts.transform.rotation.y
                  << " z: " << ts.transform.rotation.z
                  << " w: " << ts.transform.rotation.w
                  << " axis: " << q.getAxis()[0] << " " << q.getAxis()[1] << " " << q.getAxis()[2]
                  << " angle: " << q.getAngle()
                  << " " << ts.child_frame_id << std::endl;
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

    tf2::Quaternion quaternionAverage(std::vector<std::tuple<double, double, double, double, int, int>> quaternions)
{

	// first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
	Eigen::Matrix4f A = Eigen::Matrix4f::Zero();
    Eigen::Vector4f q_i = Eigen::Vector4f::Zero();
    int w_i = 0;
    int w_sum = 0;
    int quat_size = static_cast<int>(quaternions.size());

	for (int i=0; i<quat_size; i++)
    {
        q_i(0) = std::get<0>(quaternions[i]);
        q_i(1) = std::get<1>(quaternions[i]);
        q_i(2) = std::get<2>(quaternions[i]);
        q_i(3) = std::get<3>(quaternions[i]);
        w_i = std::get<4>(quaternions[i]);
		A += w_i * q_i * q_i.transpose();
        w_sum += w_i;
    }

	// normalise with the sum of the weights
	A /= w_sum;

	// // Compute the SVD of this 4x4 matrix
	// Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Eigen::VectorXf singularValues = svd.singularValues();
	// Eigen::MatrixXf U = svd.matrixU();

    // Calculate eigenvector and eigenvalues
    Eigen::EigenSolver<Eigen::Matrix4f> es(A);
    Eigen::VectorXf eigenValues = es.eigenvalues().real();
    Eigen::MatrixXf eigenVectors = es.eigenvectors().real();


	// find the eigen vector corresponding to the largest eigen value
	int largestEigenValueIndex;
	float largestEigenValue;
	bool first = true;

	for (int i=0; i<eigenValues.rows(); ++i)
	{
		if (first)
		{
			largestEigenValue = eigenValues(i);
			largestEigenValueIndex = i;
			first = false;
		}
		else if (eigenValues(i) > largestEigenValue)
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

    void tf_callback(tf2_msgs::msg::TFMessage::ConstSharedPtr tf_msg)
    {
        // std::cout << "Entra in tf_callback" << std::endl;

        const tf2_msgs::msg::TFMessage &msg_in = *tf_msg;

        if (msg_in.transforms.size() != 0)
        {
            int lower_frame_ID = 9999;
            int lower_msg_ID = 0;

            // vector of tuple: (x y z w weight frame_id)
            std::vector<std::tuple<double, double, double, double, int, int>> tuple_in;

            if (debug_)
            {
                std::cout << "-------------------------------------------------" << std::endl
                          << "Nuovo messaggio ricevuto, elenco dei quaternioni:" << std::endl;
            }

            // for every transform in the message received
            for (size_t i = 0u; i < msg_in.transforms.size(); i++)
            {
                geometry_msgs::msg::TransformStamped ts_in = msg_in.transforms[i];
                int child_id_num = std::stoi(ts_in.child_frame_id.substr(5, 4));

                if (debug_)
                    print_quaternion(ts_in);

                // push back in the tuple vector
                tuple_in.push_back(std::make_tuple(
                    ts_in.transform.rotation.x,
                    ts_in.transform.rotation.y,
                    ts_in.transform.rotation.z,
                    ts_in.transform.rotation.w,
                    get_weight(child_id_num),
                    child_id_num));

                // save lower frame number and index
                if (child_id_num < lower_frame_ID)
                {
                    lower_frame_ID = child_id_num;
                    lower_msg_ID = i;
                }
            }

            if (debug_)
                std::cout << "-------------------------------------------------" << std::endl;

            // Sort by the first element x
            std::sort(tuple_in.begin(), tuple_in.end());

            if (debug_)
            {
                std::cout << "Sorted Vector of Tuple on basis"
                             " of first element of tuple:\n";
                for (int i = 0; i < static_cast<int>(tuple_in.size()); i++)
                    std::cout << std::get<0>(tuple_in[i]) << " "
                              << std::get<1>(tuple_in[i]) << " "
                              << std::get<2>(tuple_in[i]) << " "
                              << std::get<3>(tuple_in[i]) << " "
                              << std::get<4>(tuple_in[i]) << " "
                              << std::get<5>(tuple_in[i]) << std::endl;
            }

            // Compute the weighted median and eraser the outliers
            int sum = 0;
            int all_weight = 0;
            int N = static_cast<int>(tuple_in.size());

            for (int i = 0; i < N; i++)
            {
                all_weight += std::get<4>(tuple_in[i]);
            }

            for (int i = 0; i < N; i++)
            {
                sum += std::get<4>(tuple_in[i]);
                if ((sum < all_weight / 4) || (sum > all_weight * 3 / 4))
                    tuple_in.erase(tuple_in.begin() + i);
            }

            if (debug_)
            {
                std::cout << "Sorted Vector of Tuple after deleting the outliers: \n";
                for (int i = 0; i < static_cast<int>(tuple_in.size()); i++)
                    std::cout << std::get<0>(tuple_in[i]) << " "
                              << std::get<1>(tuple_in[i]) << " "
                              << std::get<2>(tuple_in[i]) << " "
                              << std::get<3>(tuple_in[i]) << " "
                              << std::get<4>(tuple_in[i]) << " "
                              << std::get<5>(tuple_in[i]) << std::endl;
            }

            // Average quaternions
            tf2::Quaternion quat_average = quaternionAverage(tuple_in);
            
            // Average translation DA FARE


        




            // Compute the inverse is now done in apriltag_ros
            // tf2::Transform trans(tf2::Quaternion(
            //                          t.transform.rotation.x,
            //                          t.transform.rotation.y,
            //                          t.transform.rotation.z,
            //                          t.transform.rotation.w),
            //                      tf2::Vector3(
            //                          t.transform.translation.x,
            //                          t.transform.translation.y,
            //                          t.transform.translation.z));

            // trans = trans.inverse();

            // Do some maths to get the correct transform from the world reference system

            // camera_origin[0] = trans.getOrigin().x();
            // camera_origin[1] = trans.getOrigin().y();
            // camera_origin[2] = trans.getOrigin().z();

            // quat_cam = trans.getRotation();


            // Per usare il frame di dimensione maggiore
            t = msg_in.transforms[lower_msg_ID];

            camera_origin[0] = t.transform.translation.x;
            camera_origin[1] = t.transform.translation.y;
            camera_origin[2] = t.transform.translation.z;

            quat_cam[0] = t.transform.rotation.x;
            quat_cam[1] = t.transform.rotation.y;
            quat_cam[2] = t.transform.rotation.z;
            quat_cam[3] = t.transform.rotation.w;

            quat_body = quat_cam * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;

            body_origin = quatRotate(quat_cam, offset_camera_body_vect_) + camera_origin + get_offset(lower_frame_ID);

            if (debug_)
            {
                std::cout << "t: s = " << t.header.stamp.sec << ", ns = " << t.header.stamp.nanosec << ", frame_id = " << t.header.frame_id
                          << ", child_frame_id = " << t.child_frame_id << std::endl
                          << "transform: x = " << t.transform.translation.x
                          << ", y = " << t.transform.translation.y << ", z = " << t.transform.translation.z << std::endl
                          << std::endl
                          << std::endl;
            }

            // calculate the delta time between PX4 and t messages for synchronizations
            if (time_start == 0 || t.header.stamp.sec == 0)
            {
                // update time_start from timestamps_ (SYNCED)
                time_start = static_cast<u_int64_t>(timestamp_.load());
                std::cout << "timestamp " << time_start << std::endl;
                delta_time = time_start - static_cast<u_int64_t>(t.header.stamp.sec * 1000000 + t.header.stamp.nanosec / 1000);
                RCLCPP_INFO(this->get_logger(), "Time start at time: %d and the delta time for synchronization is: %d", time_start, delta_time);
            }
            else
            {

                // Generate the message
                msg.timestamp = time_start; // time since system start (microseconds)
                msg.timestamp_sample = t.header.stamp.sec * 1000000 + t.header.stamp.nanosec / 1000 + delta_time;
                msg.local_frame = 0; // LOCAL_FRAME_NED=0         # NED earth-fixed frame
                // msg.local_frame = 1;	//FRD earth-fixed frame, arbitrary heading reference
                msg.x = body_origin.getX();
                msg.y = body_origin.getY();
                msg.z = body_origin.getZ();

                // VehicleVisualOdometry msg has quaternion defined like: q{w, x, y, z} = {scalar, vect(3)}
                // tf2::Quaternion is defined like: q{x, y, z, w} = {vect(3), scalar}
                msg.q.at(0) = quat_body[3];
                msg.q.at(1) = quat_body[0];
                msg.q.at(2) = quat_body[1];
                msg.q.at(3) = quat_body[2];
                msg.q_offset.at(0) = NAN;
                msg.pose_covariance.at(0) = NAN;
                msg.pose_covariance.at(15) = NAN;

                msg.velocity_frame = 0;
                msg.vx = NAN;
                msg.vy = NAN;
                msg.vz = NAN;
                msg.rollspeed = NAN;
                msg.pitchspeed = NAN;
                msg.yawspeed = NAN;
                msg.velocity_covariance.at(0) = NAN;
                msg.velocity_covariance.at(15) = NAN;

                if (debug_)
                {
                    std::cout << "POSE 19-08-2022" << std::endl;
                    std::cout << "\t translations: [ x: " << msg.x << ", y: " << msg.y << ", z: " << msg.z << " ]" << std::endl;
                    std::cout << "\t quaternion: [ w: " << msg.q[0] << ", ( x: " << msg.q[1] << ", y: " << msg.q[2] << ", z: " << msg.q[3] << ") ]" << std::endl
                              << std::endl;
                }
                if (graphics_on_)
                {
                    rclcpp::Time now = this->get_clock()->now();
                    geometry_msgs::msg::TransformStamped tf_drone;

                    tf_drone.header.stamp = now;
                    tf_drone.header.frame_id = home_map_frame_;
                    tf_drone.child_frame_id = drone_frame_estimated_;

                    tf_drone.transform.translation.x = msg.x;
                    tf_drone.transform.translation.y = msg.y;
                    tf_drone.transform.translation.z = msg.z;

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

    // For a better visualization create two home one with z_up (home_rviz) and one with z_down (home_map)
    std::string home_map_frame_ = "home_map";   // z_down
    std::string home_rviz_frame_ = "home_rviz"; // z_up

    std::string drone_frame_estimated_ = "drone";

    // Parameter from yaml file
    bool debug_;
    bool graphics_on_;
    std::string fromFrameRel_;
    float roll_cam_;                       //[deg]
    float pitch_cam_;                      //[deg]
    float yaw_cam_;                        //[deg]
    tf2::Vector3 offset_camera_body_vect_; //[m]
    tf2::Vector3 offset_body_camera_vect_; //[m]
    std::vector<int64_t> tag_ids_;
    std::vector<int> frame_weight;

    std::vector<tf2::Vector3> tags_locations_XL_;
    std::vector<tf2::Vector3> tags_locations_L_;
    std::vector<tf2::Vector3> tags_locations_M_;
    std::vector<tf2::Vector3> tags_locations_S_;

    geometry_msgs::msg::TransformStamped t;
    px4_msgs::msg::VehicleVisualOdometry msg;

    tf2::Quaternion quat_cam_to_body_x, quat_cam_to_body_y, quat_cam_to_body_z;
    tf2::Quaternion quat_body; // from apriltag-frame to body-frame
    tf2::Quaternion quat_cam;  // from apriltag-frame to camera-frame
    tf2::Vector3 camera_origin, body_origin;
};

int main(int argc, char *argv[])
{
    std::cout << "Starting apriltag_to_visual_odometry node..." << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
