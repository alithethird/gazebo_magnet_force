
#ifndef INCLUDE_MAC_GAZEBO_MAGNET_FORCE_MAGNET_FORCE_H_
#define INCLUDE_MAC_GAZEBO_MAGNET_FORCE_MAGNET_FORCE_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
/*
plugin olmasi icin gerekli dosyalar eklendi 
*/
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/MagneticField.h>
/*
    calismasi icin gerekli dosyalar eklendi 
*/
#include <memory>

#include "gazebo_magnet_force/magnet_force_container.h"

namespace gazebo
{
    class MagnetForce : public ModelPlugin
    {
    public:
        MagnetForce();

        ~MagnetForce();

        /// \brief Loads the plugin
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        /// \brief Callback for when subscribers connect
        void Connect();

        /// \brief Callback for when subscribers disconnect
        void Disconnect();

        /// \brief Thread to interact with ROS
        void QueueThread();

        /// \brief Called by the world update start event
        void OnUpdate(const common::UpdateInfo & /*_info*/);

        /// \brief Publishes data to ros topics
        /// \pram[in] force A vector of force that makes up the wrench to be published
        /// \pram[in] torque A vector of torque that makes up the wrench to be published
        /// \pram[in] mfs A vector of magnetic field data
        void PublishData(
            const ignition::math::Vector3d& force,
            const ignition::math::Vector3d& torque,
            const ignition::math::Vector3d& mfs);

        /// \brief Calculate force and torque of a magnet on another
        /// \parama[in] p_self Pose of the first magnet
        /// \parama[in] m_self Dipole moment of the first magnet
        /// \parama[in] p_other Pose of the second magnet
        /// \parama[in] m_other Dipole moment of the second magnet on which the force is calculated
        /// \param[out] force Calculated force vector
        /// \param[out] torque Calculated torque vector
        void GetForceTorque(const ignition::math::Pose3d &p_self, const ignition::math::Vector3d &m_self,
                            const ignition::math::Pose3d &p_other, const ignition::math::Vector3d &m_other,
                            ignition::math::Vector3d &force, ignition::math::Vector3d &torque);

        /// \brief Calculate the magnetic field on all 6 sensors
        /// \parama[in] p_self Pose of the first magnet
        /// \parama[in] p_other Pose of the second magnet
        /// \parama[in] m_other Dipole moment of the second magnet
        /// \param[out] mfs magnetic field sensors
        void GetMFS(const ignition::math::Pose3d &p_self,
                    const ignition::math::Pose3d &p_other,
                    const ignition::math::Vector3d &m_other,
                    ignition::math::Vector3d &mfs);

        // Pointer to the model
    private:
        physics::ModelPtr model;
        physics::LinkPtr link;
        physics::WorldPtr world;

        std::shared_ptr<MagnetForceContainer::Magnet> mag;

        std::string link_name;
        std::string robot_namespace;
        std::string topic_ns;

        bool should_publish;
        ros::NodeHandle *rosnode;
        ros::Publisher wrench_pub;
        ros::Publisher mfs_pub;

        geometry_msgs::WrenchStamped wrench_msg;
        sensor_msgs::MagneticField mfs_msg;

    private:
        boost::mutex lock;
        int connect_count;

        // Custom Callback Queue
        ros::CallbackQueue queue;
        boost::thread callback_queue_thread;

        common::Time last_time;
        double update_rate;
        // Pointer to the update event connection
        event::ConnectionPtr update_connection;
    };
} // namespace gazebo

#endif
