#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>

std::string target_frame = "link1";
std::string source_frame = "flange";


int main(int argc, char** argv) {
    geometry_msgs::Transform transform;     // Current transform
    geometry_msgs::Vector3 v;                          // Transform translation
                           // Transform rotation
    tf2::Vector3 axis;                       // Axis of axis-angle represenentation of transform rotation7
    tf2Scalar angle;                        // Angle of axis-angle representation of transform rotation
    tf2Scalar roll, pitch, yaw;             // Angles of RPY representation of transform rotation 
    std_msgs::String msg;                   // Message to log
    
    // Init node
    ros::init(argc, argv, "fanuc_listener");
    ros::NodeHandle nodeHandle;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Wait for the first transform to become avaiable for up to 5 seconds.
    try{
        tfBuffer.lookupTransform(target_frame, source_frame, ros::Time::now(), ros::Duration(5.0));
    } catch (tf2::TransformException &ignored) { }

    // Start the main loop
    ros::Rate rate(0.1);
    while (nodeHandle.ok()) {

        // Look for transforms
        try {
            for (int l=1; l<=6; l++) {
                std::cout << std::endl << "********** Link " << l << " to flange ***********" << std::endl;

                std::stringstream sst;
                sst << "link" << l;
                target_frame = sst.str();
                
                // Get transform message
                transform = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0)).transform;
                v = transform.translation;
                geometry_msgs::Quaternion rotation = transform.rotation;
                tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);

                // Convert message to different representations
                axis = q.getAxis();            
                angle = q.getAngle();
                tf2::Matrix3x3 rotmatrix(q);
                rotmatrix.getRPY(roll, pitch, yaw);
                
                // Cout the transform
                std::stringstream ss;
                ss << "- Translation: [" << v.x << ", " << v.y << ", " << v.z << "]" << std::endl;
                ss << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << "]" << std::endl;
                ss << "            in Axis-Angle [" << axis.getX() << ", " << axis.getY() << ", " << axis.getZ() << "], " << angle << std::endl;
                ss << "            in RPY [" << roll << ", " << pitch << ", " << yaw << "]" << std::endl;
                ss << "            Rotation matrix: " << std::endl;
                for (int i=0; i<3; i++) {
                    ss << "            [" << rotmatrix[i][0] << ", " << rotmatrix[i][1] << ", " << rotmatrix[i][2] << "]" << std::endl;
                }
                msg.data = ss.str();
                std::cout << msg.data.c_str();
            }
        } catch (tf2::TransformException &exception) {
            std::stringstream ss;
            ss << "Could NOT transform " << source_frame << " to " << target_frame;
            ROS_WARN("%s: %s", ss.str().c_str(), exception.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }

    return 0;
}