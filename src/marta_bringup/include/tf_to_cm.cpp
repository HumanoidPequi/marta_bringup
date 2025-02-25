#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <urdf/model.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <map>
#include <mutex>

// Variável global para armazenar os estados das juntas e seu mutex
std::map<std::string, double> g_joint_states;
std::mutex joint_state_mutex;

// Callback que atualiza o mapa de estados das juntas
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex);
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    g_joint_states[msg->name[i]] = msg->position[i];
  }
}

// Função auxiliar: converte um urdf::Pose em tf2::Transform
tf2::Transform poseToTF(const urdf::Pose &pose)
{
  double qx, qy, qz, qw;
  pose.rotation.getQuaternion(qx, qy, qz, qw);
  tf2::Quaternion q(qx, qy, qz, qw);
  tf2::Vector3 origin(pose.position.x, pose.position.y, pose.position.z);
  return tf2::Transform(q, origin);
}

// Função recursiva para computar a transformação global de cada link,
// agora levando em conta o estado atual da junta (se houver) para atualizar
// a cinemática e, consequentemente, o COM.
void computeLinkTransforms(const urdf::LinkConstSharedPtr &link,
                           const tf2::Transform &parent_tf,
                           std::vector<std::pair<double, tf2::Vector3> > &mass_positions,
                           const urdf::Model &model,
                           const std::map<std::string, double> &joint_states)
{
  // A transformação do link é a do pai
  tf2::Transform current_tf = parent_tf;

  // Se o link possui dados inerciais, computa a posição global do seu COM
  if (link->inertial)
  {
    tf2::Transform inertial_tf = poseToTF(link->inertial->origin);
    tf2::Transform link_com_tf = current_tf * inertial_tf;
    mass_positions.push_back(std::make_pair(link->inertial->mass, link_com_tf.getOrigin()));
  }

  // Para cada junta que conecta este link a um link filho:
  for (const auto &child_joint : link->child_joints)
  {
    if (!child_joint)
      continue;

    // Começamos com a transformação definida no URDF para a junta
    tf2::Transform joint_tf = poseToTF(child_joint->parent_to_joint_origin_transform);

    // Se a junta é revoluta ou contínua, incorpora o ângulo atual
    if (child_joint->type == urdf::Joint::REVOLUTE ||
        child_joint->type == urdf::Joint::CONTINUOUS)
    {
      double angle = 0.0;
      auto it = joint_states.find(child_joint->name);
      if (it != joint_states.end())
        angle = it->second;

      tf2::Quaternion q;
      tf2::Vector3 axis(child_joint->axis.x, child_joint->axis.y, child_joint->axis.z);
      q.setRotation(axis, angle);
      tf2::Transform joint_motion(q, tf2::Vector3(0, 0, 0));
      joint_tf = joint_tf * joint_motion;
    }
    // Se for prismatic, utiliza o deslocamento atual
    else if (child_joint->type == urdf::Joint::PRISMATIC)
    {
      double displacement = 0.0;
      auto it = joint_states.find(child_joint->name);
      if (it != joint_states.end())
        displacement = it->second;

      tf2::Vector3 translation(child_joint->axis.x * displacement,
                               child_joint->axis.y * displacement,
                               child_joint->axis.z * displacement);
      tf2::Transform joint_motion;
      joint_motion.setIdentity();
      joint_motion.setOrigin(translation);
      joint_tf = joint_tf * joint_motion;
    }

    // Computa a transformação global do link filho
    tf2::Transform child_tf = current_tf * joint_tf;
    urdf::LinkConstSharedPtr child_link = model.getLink(child_joint->child_link_name);
    if (child_link)
      computeLinkTransforms(child_link, child_tf, mass_positions, model, joint_states);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_and_com_node");
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster tf_broadcaster;

  // Carrega o URDF a partir do parâmetro "robot_description"
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
  {
    ROS_ERROR("Falha ao carregar o URDF do parâmetro 'robot_description'");
    return -1;
  }
  ROS_INFO("URDF carregado com sucesso.");

  // Inscreve-se em /marta/joint_states (sobrescrevendo os valores conforme forem chegando)
  ros::Subscriber joint_state_sub = nh.subscribe("/marta/joint_states", 10, jointStateCallback);

  ros::Rate rate(10.0);
  while (ros::ok())
  {
    // Copia os estados atuais das juntas (protege com mutex)
    std::map<std::string, double> current_joint_states;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex);
      current_joint_states = g_joint_states;
    }

    // Recalcula o COM usando os ângulos atuais
    std::vector<std::pair<double, tf2::Vector3> > mass_positions;
    tf2::Transform identity;
    identity.setIdentity();

    urdf::LinkConstSharedPtr root_link = urdf_model.getRoot();
    if (!root_link)
    {
      ROS_ERROR("Não foi possível identificar o link raiz no URDF.");
      return -1;
    }

    computeLinkTransforms(root_link, identity, mass_positions, urdf_model, current_joint_states);

    // Soma ponderada para calcular o COM
    double total_mass = 0.0;
    tf2::Vector3 weighted_sum(0, 0, 0);
    for (const auto &m_pair : mass_positions)
    {
      total_mass += m_pair.first;
      weighted_sum += m_pair.first * m_pair.second;
    }

    tf2::Vector3 center_of_mass(0, 0, 0);
    if (total_mass > 0)
    {
      center_of_mass = weighted_sum / total_mass;
      // ROS_INFO("COM atualizado: x=%.3f, y=%.3f, z=%.3f", center_of_mass.x(), center_of_mass.y(), center_of_mass.z());
    }
    else
    {
      ROS_WARN("Massa total zero. Não foi possível calcular o COM.");
    }

    ros::Time now = ros::Time::now();

    // Publica o TF para o l_feet_point com pai em l_feet_1
    geometry_msgs::TransformStamped l_feet_tf;
    l_feet_tf.header.stamp = now;
    l_feet_tf.header.frame_id = "l_feet_1";
    l_feet_tf.child_frame_id = "l_feet_point";
    l_feet_tf.transform.translation.x = 0.02445;
    l_feet_tf.transform.translation.y = 0.0;
    l_feet_tf.transform.translation.z = -0.037;
    l_feet_tf.transform.rotation.x = 0.0;
    l_feet_tf.transform.rotation.y = 0.0;
    l_feet_tf.transform.rotation.z = 0.0;
    l_feet_tf.transform.rotation.w = 1.0;

    // Publica o TF para o r_feet_point com pai em r_feet_1
    geometry_msgs::TransformStamped r_feet_tf;
    r_feet_tf.header.stamp = now;
    r_feet_tf.header.frame_id = "r_feet_1";
    r_feet_tf.child_frame_id = "r_feet_point";
    r_feet_tf.transform.translation.x = 0.02445;
    r_feet_tf.transform.translation.y = 0.0;
    r_feet_tf.transform.translation.z = -0.037;
    r_feet_tf.transform.rotation.x = 0.0;
    r_feet_tf.transform.rotation.y = 0.0;
    r_feet_tf.transform.rotation.z = 0.0;
    r_feet_tf.transform.rotation.w = 1.0;

    // Publica o TF para o centro de massa com pai no link raiz (por exemplo, "base_link")
    geometry_msgs::TransformStamped com_tf;
    com_tf.header.stamp = now;
    com_tf.header.frame_id = root_link->name;  // ou "base_link", conforme seu URDF
    com_tf.child_frame_id = "center_of_mass";
    com_tf.transform.translation.x = center_of_mass.x();
    com_tf.transform.translation.y = center_of_mass.y();
    com_tf.transform.translation.z = center_of_mass.z();
    com_tf.transform.rotation.x = 0.0;
    com_tf.transform.rotation.y = 0.0;
    com_tf.transform.rotation.z = 0.0;
    com_tf.transform.rotation.w = 1.0;

    // Envia os transforms
    tf_broadcaster.sendTransform(l_feet_tf);
    tf_broadcaster.sendTransform(r_feet_tf);
    tf_broadcaster.sendTransform(com_tf);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
