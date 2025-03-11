#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <urdf/model.h>

// Estrutura global para armazenar o modelo URDF
urdf::Model robot_model;

// Função auxiliar para criar uma transformação
void publishTransform(const std::string& parent_frame, const std::string& child_frame,
                      double roll, double pitch, double yaw, tf2_ros::TransformBroadcaster& broadcaster) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;

    // Pega o link do modelo URDF
    urdf::LinkConstSharedPtr child_link = robot_model.getLink(child_frame);
    if (child_link && child_link->parent_joint) {
        // Pega a posição do link
        transform.transform.translation.x = child_link->parent_joint->parent_to_joint_origin_transform.position.x;
        transform.transform.translation.y = child_link->parent_joint->parent_to_joint_origin_transform.position.y;
        transform.transform.translation.z = child_link->parent_joint->parent_to_joint_origin_transform.position.z;
    } else {
        ROS_WARN("Link %s não encontrado no URDF ou não possui junta associada.", child_frame.c_str());
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
    }

    // Cria o quaternion com base nos ângulos roll, pitch e yaw
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    broadcaster.sendTransform(transform);
}

// Callback para o estado da perna direita
void rightLegCallback(const std_msgs::Float64MultiArray::ConstPtr& msg, tf2_ros::TransformBroadcaster& broadcaster) {
    publishTransform("base_link", "r_hip_1",   0, 0, msg->data[0], broadcaster);         // Hip yaw
    publishTransform("r_hip_1", "r_waist_1",   msg->data[1], 0, 0, broadcaster);        // Hip roll
    publishTransform("r_waist_1", "r_thigh_1", 0, msg->data[2], 0, broadcaster);      // Hip pitch
    publishTransform("r_thigh_1", "r_shin_1",  0, msg->data[3], 0, broadcaster);       // Knee pitch
    publishTransform("r_shin_1", "r_ankle_1",  0, msg->data[4], 0, broadcaster);       // Ankle pitch
    publishTransform("r_ankle_1", "r_feet_1",  msg->data[5], 0, 0, broadcaster);       // Ankle roll
}

// Callback para o estado da perna esquerda
void leftLegCallback(const std_msgs::Float64MultiArray::ConstPtr& msg, tf2_ros::TransformBroadcaster& broadcaster) {
    publishTransform("base_link", "l_hip_1",   0,            0,             msg->data[0],              broadcaster);         // Hip yaw
    publishTransform("l_hip_1", "l_waist_1",   msg->data[1], 0,             0,              broadcaster);        // Hip roll
    publishTransform("l_waist_1", "l_thigh_1", 0,            msg->data[2],  0,              broadcaster);      // Hip pitch
    publishTransform("l_thigh_1", "l_shin_1",  0,            msg->data[3],  0,              broadcaster);       // Knee pitch
    publishTransform("l_shin_1", "l_ankle_1",  0,            msg->data[4],  0,              broadcaster);       // Ankle pitch
    publishTransform("l_ankle_1", "l_feet_1",  msg->data[5], 0,             0,              broadcaster);       // Ankle roll
}

// Callback para o estado do braço direito
void rightArmCallback(const std_msgs::Float64MultiArray::ConstPtr& msg, tf2_ros::TransformBroadcaster& broadcaster) {
    publishTransform("base_link", "r_shoulder_1", 0, msg->data[0], 0, broadcaster);      // Shoulder pitch
    publishTransform("r_shoulder_1", "r_arm_1", -msg->data[1], 0, 0, broadcaster);        // Shoulder roll
    publishTransform("r_arm_1", "r_hand_1", 0, msg->data[2], 0, broadcaster);           // Elbow pitch
}

// Callback para o estado do braço esquerdo e cabeça
void leftArmHeadCallback(const std_msgs::Float64MultiArray::ConstPtr& msg, tf2_ros::TransformBroadcaster& broadcaster) {
    publishTransform("base_link", "neck_1", 0, 0, msg->data[0], broadcaster);           // Head pan
    publishTransform("neck_1", "head_1", 0, msg->data[1], 0, broadcaster);              // Head tilt
    publishTransform("base_link", "l_shoulder_1", 0, msg->data[2], 0, broadcaster);     // Shoulder pitch
    publishTransform("l_shoulder_1", "l_arm_1", -msg->data[3], 0, 0, broadcaster);       // Shoulder roll
    publishTransform("l_arm_1", "l_hand_1", 0, msg->data[4], 0, broadcaster);           // Elbow pitch
}

// Função para publicar uma transformação estática
void publishStaticTransform(const std::string& parent_frame, const std::string& child_frame, double x, double y, double z, tf2_ros::StaticTransformBroadcaster& broadcaster) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;

    // Quaternion sem rotação
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    broadcaster.sendTransform(transform);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "marta_tf_broadcaster");
    ros::NodeHandle nh;

    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // Carrega o modelo URDF
    if (!robot_model.initParam("/robot_description")) {
        ROS_ERROR("Falha ao carregar o modelo URDF do parâmetro /robot_description");
        return -1;
    }

    // Publicação estática para imu_position
    publishStaticTransform("base_link", "imu_position", -5.76e-3, 53.277e-3, 48.948e-3, static_broadcaster);

    // Subscrições dos tópicos
    ros::Subscriber right_leg_sub = nh.subscribe<std_msgs::Float64MultiArray>(
        "/marta/right_leg/state", 10,
        boost::bind(&rightLegCallback, _1, boost::ref(broadcaster)));

    ros::Subscriber left_leg_sub = nh.subscribe<std_msgs::Float64MultiArray>(
        "/marta/left_leg/state", 10,
        boost::bind(&leftLegCallback, _1, boost::ref(broadcaster)));

    ros::Subscriber right_arm_sub = nh.subscribe<std_msgs::Float64MultiArray>(
        "/marta/arm_r/state", 10,
        boost::bind(&rightArmCallback, _1, boost::ref(broadcaster)));

    ros::Subscriber left_arm_head_sub = nh.subscribe<std_msgs::Float64MultiArray>(
        "/marta/arm_l_head/state", 10,
        boost::bind(&leftArmHeadCallback, _1, boost::ref(broadcaster)));

    ros::spin();
    return 0;
}
