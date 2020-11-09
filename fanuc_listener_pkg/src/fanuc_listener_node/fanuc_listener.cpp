#include <math.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/String.h>


std::string target_frame = "link1";
std::string source_frame = "flange";


int main(int argc, char** argv) {
    geometry_msgs::Transform transform;     // Current transform
    geometry_msgs::Vector3 v;               // Transform translation
    tf2::Quaternion q;                      // Transform rotation
    tf2::Vector3 axis;                      // Axis of axis-angle represenentation of transform rotation7
    tf2Scalar angle;                        // Angle of axis-angle representation of transform rotation
    tf2Scalar roll, pitch, yaw;             // Angles of RPY representation of transform rotation 
    
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
            std::stringstream ss;
            ss << std::endl;
            // Cycle through all links
            for (int l=1; l<=6; l++) {
                // Print link information
                ss << std::endl << "************ Transformation from link" << l << " to end-effector ************" << std::endl << std::endl;

                // Assign current link
                std::stringstream linkNameStringStream;
                linkNameStringStream << "link" << l;
                target_frame = linkNameStringStream.str();
                
                // Get transform message
                transform = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0)).transform;
                v = transform.translation;
                tf2::convert(transform.rotation, q);

                // Convert message to different representations
                axis = q.getAxis();            
                angle = q.getAngle();
                tf2::Matrix3x3 rotmatrix(q);
                for (int i=0; i<3; i++)
                    for (int j=0; j<3; j++)
                        rotmatrix[i][j] = round(rotmatrix[i][j] * 10000.0) / 10000.0;
                rotmatrix.getRPY(roll, pitch, yaw);
                
                // Cout the transform
                ss << "------- Translation -------" << std::endl;
                ss << "x: " << v.x << std::endl << "y: " << v.y <<std::endl << "z: " << v.z << std::endl << std::endl;
                ss << "------- Quaternion -------" << std::endl;
                ss << "[" << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << "]" << std::endl << std::endl;
                ss << "------- Axis/angle -------" << std::endl;
                ss << "Axis = [" << axis.getX() << ", " << axis.getY() << ", " << axis.getZ() << "]" << std::endl;
                ss << "Angle = " << angle << std::endl << std::endl;
                ss << "------- Rotation matrix -------" << std::endl;
                for (int i=0; i<3; i++)
                    ss << "[" << rotmatrix[i][0] << ", " << rotmatrix[i][1] << ", " << rotmatrix[i][2] << "]" << std::endl;
                ss << std::endl << "------- Euler angles (RPY) -------" << std::endl;
                ss << "[" << roll << ", " << pitch << ", " << yaw << "]" << std::endl;
            }
            ROS_INFO("%s", ss.str().c_str());
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