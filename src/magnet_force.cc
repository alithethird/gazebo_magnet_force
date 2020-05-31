#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
//bu library'yi bilmiyorum ama lazim heralde
//sonradan silinebilir simdilik dursun
//baya lazimmis silinemez
//gazebo eventleri icin kullaniliyor

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>
// ros ile integrasyon icin gerekli
#include <iostream>
#include <vector>
#include <cstdint>
// klasik cpp kutuphaneleri

#include "gazebo_magnet_force/magnet_force.h"

namespace gazebo
{
    MagnetForce::MagnetForce() : ModelPlugin()
    {
        this->connect_count = 0;
    }

    MagnetForce::~MagnetForce()
    {
        this->update_connection.reset();
        if (this->should_publish)
        {
            this->queue.clear();
            this->queue.disable();
            this->rosnode->shutdown();
            this->callback_queue_thread.join();
            delete this->rosnode;
            //bura sistem kapanirken ros nodedan kopmak icin sanirim
        }
        if (this->mag)
        {
            MagnetForceContainer::Get().Remove(this->mag);
            //we need container after all
            // container yazdim(copy paste xD)
        }
    }

    void MagnetForce::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // store the pointer to the model
        this->model = _parent;
        this->world = _parent->GetWorld();
        gzdbg << "Loading MagnetForce plugin" << std::endl;

        this->mag = std::make_shared<MagnetForceContainer::Magnet>();

        //load parameters
        this->robot_namespace = "";             // robot namespace olustur
        if (_sdf->HasElement("robotNamespace")) //eger sdf dosyasindaki modelin robot namespacei varsa bunu elimizdekine yaz
            this->robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
        if (!_sdf->HasElement("bodyName"))
        {
            gzerr << "Magnet Force plugin missing <bodyname>, cannot proceed" << std::endl
                  << "MagnetForce plugini robotta isim bulamadi devam edemiyor (sdf'e bodyName ekleyin)" << std::endl;
            return;
        }
        else
            this->link_name = _sdf->GetElement("bodyName")->Get<std::string>();

        this->link = this->model->GetLink(this->link_name);
        if (!this->link)
        {
            gzerr << "Error: link named" << this->link_name << "does no exist" << std::endl;
            return;
        }

        this->should_publish = false;
        if (_sdf->HasElement("shouldPublish"))
            this->should_publish = _sdf->GetElement("shouldPublish")->Get<bool>();

        if (!_sdf->HasElement("updateRate"))
        {
            gzmsg << "MagnetForce plugin missing <updateRate>, defaults to 0.0"
                     "(as fast as possible)"
                  << std::endl;
            this->update_rate = 0;
        }
        else
            this->update_rate = _sdf->GetElement("updateRate")->Get<double>();

        if (_sdf->HasElement("calculate"))
        {
            this->mag->calculate = _sdf->Get<bool>("calculate");
        }
        else
            this->mag->calculate = true;

        if (_sdf->HasElement("dipole_moment"))
            this->mag->moment = _sdf->Get<ignition::math::Vector3d>("dipole_moment");

        if (_sdf->HasElement("xyzOffset"))
            this->mag->offset.Pos() = _sdf->Get<ignition::math::Vector3d>("xyzOffset");

        if (_sdf->HasElement("rpyOffset"))
        {
            ignition::math::Vector3d rpy_offset = _sdf->Get<ignition::math::Vector3d>("rpyOffset");
            this->mag->offset.Rot() = ignition::math::Quaterniond(rpy_offset);
        }

        if (this->should_publish)
        {
            if (!_sdf->HasElement("topicNs"))
            {
                gzmsg << "MagnetForce plugin missing <topicNs>,"
                         "will publish on namespace "
                      << this->link_name << std::endl;
            }
            else
                this->topic_ns = _sdf->GetElement("topicNs")->Get<std::string>();

            if (!ros::isInitialized())
            {
                gzerr << "A ROS node for Gazebo has not been initialized, unable to load "
                         "plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
                         "the gazebo_ros package. If you want to use this plugin without ROS, "
                         "set <shouldPublish> to false"
                      << std::endl;
                return;
            }

            this->rosnode = new ros::NodeHandle(this->robot_namespace);
            this->rosnode->setCallbackQueue(&this->queue);

            this->wrench_pub = this->rosnode->advertise<geometry_msgs::WrenchStamped>(
                this->topic_ns + "/wrench", 1,
                boost::bind(&MagnetForce::Connect, this),
                boost::bind(&MagnetForce::Disconnect, this), ros::VoidPtr(), &this->queue);
            this->mfs_pub = this->rosnode->advertise<sensor_msgs::MagneticField>(
                this->topic_ns + "/mfs", 1,
                boost::bind(&MagnetForce::Connect, this),
                boost::bind(&MagnetForce::Disconnect, this), ros::VoidPtr(), &this->queue);
            //custom callback queue
            this->callback_queue_thread = boost::thread(boost::bind(&MagnetForce::QueueThread, this));
        }

        this->mag->model_id = this->model->GetId();

        gzmsg << "Loaded Gazebo magnet force plugin on " << this->model->GetName() << std::endl;

        MagnetForceContainer::Get().Add(this->mag);

        //listen to the update event. this event is broadcasted every simulation iteration
        this->update_connection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&MagnetForce::OnUpdate, this, _1));
    }

    void MagnetForce::Connect()
    {
        this->connect_count++;
    }

    void MagnetForce::Disconnect()
    {
        this->connect_count--;
    }

    void MagnetForce::QueueThread()
    {
        static const double timeout = 0.01;

        while (this->rosnode->ok())
        {
            this->queue.callAvailable(ros::WallDuration(timeout));
        }
    }

    //called by the world update start event
    void MagnetForce::OnUpdate(const common::UpdateInfo & /*_info*/)
    {

        //calculate the force from all other magnets
        ignition::math::Pose3d p_self = this->link->WorldPose();
        p_self.Pos() += -p_self.Rot().RotateVector(this->mag->offset.Pos());
        p_self.Rot() *= this->mag->offset.Rot().Inverse();

        this->mag->pose = p_self;

        if (!this->mag->calculate)
            return;

        MagnetForceContainer &mf = MagnetForceContainer::Get();

        ignition::math::Vector3d moment_world = p_self.Rot().RotateVector(this->mag->moment);

        ignition::math::Vector3d force;
        ignition::math::Vector3d torque;
        ignition::math::Vector3d mfs;
        for (MagnetForceContainer::MagnetPtrV::iterator it = mf.magnets.begin(); it < mf.magnets.end(); it++)
        {
            std::shared_ptr<MagnetForceContainer::Magnet> mag_other = *it;
            if (mag_other->model_id != this->mag->model_id)
            {
                ignition::math::Pose3d p_other = mag_other->pose;
                ignition::math::Vector3d m_other = p_other.Rot().RotateVector(mag_other->moment);
                // vektorler rotate ediliyor
                //neye gore bilmiyoum ama bi incelemede buluruz (kek)
                ignition::math::Vector3d force_tmp;
                ignition::math::Vector3d torque_tmp;
                GetForceTorque(p_self, moment_world, p_other, m_other, force_tmp, torque_tmp);
                //elimizdeki magnete etkiyen butun diger magnetler tek tek itere edilerek
                //aradaki kuvvet hesaplaniyor ve toplaniyor
                force += force_tmp;
                torque += torque_tmp;
                //elimizdeki magnete etkiyen butun diger magnetler tek tek itere edilerek
                //aradaki tork hesaplaniyor ve toplaniyor

                ignition::math::Vector3d mfs_tmp;
                GetMFS(p_self, p_other, m_other, mfs_tmp);

                mfs += mfs_tmp;

                this->link->AddForce(force_tmp);
                this->link->AddTorque(torque_tmp);
            }
        }

        this->PublishData(force, torque, mfs);
    }

    void MagnetForce::PublishData(
        const ignition::math::Vector3d &force,
        const ignition::math::Vector3d &torque,
        const ignition::math::Vector3d &mfs)
    {
        if (this->should_publish && this->connect_count > 0)
        {
            // Rate control
            common::Time cur_time = this->world->SimTime();
            if (this->update_rate > 0 &&
                (cur_time - this->last_time).Double() < (1.0 / this->update_rate))
                return;

            this->lock.lock();
            // copy data into wrench message
            this->wrench_msg.header.frame_id = "world";
            this->wrench_msg.header.stamp.sec = cur_time.sec;
            this->wrench_msg.header.stamp.nsec = cur_time.nsec;

            this->wrench_msg.wrench.force.x = force.X();
            this->wrench_msg.wrench.force.y = force.Y();
            this->wrench_msg.wrench.force.z = force.Y();
            this->wrench_msg.wrench.torque.x = torque.X();
            this->wrench_msg.wrench.torque.y = torque.Y();
            this->wrench_msg.wrench.torque.z = torque.Z();

            // now mfs
            this->mfs_msg.header.frame_id = this->link_name;
            this->mfs_msg.header.stamp.sec = cur_time.sec;
            this->mfs_msg.header.stamp.nsec = cur_time.nsec;

            this->mfs_msg.magnetic_field.x = mfs.X();
            this->mfs_msg.magnetic_field.y = mfs.Y();
            this->mfs_msg.magnetic_field.z = mfs.Z();

            this->wrench_pub.publish(this->wrench_msg);
            this->mfs_pub.publish(this->mfs_msg);

            this->lock.unlock();
        }
    }

    void MagnetForce::GetForceTorque(const ignition::math::Pose3d &p_self,
                                     const ignition::math::Vector3d &m_self,
                                     const ignition::math::Pose3d &p_other,
                                     const ignition::math::Vector3d &m_other,
                                     ignition::math::Vector3d &force,
                                     ignition::math::Vector3d &torque
                                     )
    {

        bool debug = true;
        ignition::math::Vector3d p = p_self.Pos() - p_other.Pos();
        ignition::math::Vector3d p_unit = p / p.Length();

        ignition::math::Vector3d m1 = m_other;
        ignition::math::Vector3d m2 = m_self;
        if (debug)
            std::cout << "p: " << p << " m1: " << m1 << " m2: " << m2 << std::endl;

        double K = 3.0 * 1e-7 / pow(p.Length(), 4);
        force = K * (m2 * (m1.Dot(p_unit)) + m1 * (m2.Dot(p_unit)) +
                     p_unit * (m1.Dot(m2)) - 5 * p_unit * (m1.Dot(p_unit)) * (m2.Dot(p_unit)));

        double Ktorque = 1e-7 / pow(p.Length(), 3);
        ignition::math::Vector3d B1 = Ktorque * (3 * (m1.Dot(p_unit)) * p_unit - m1);
        torque = m2.Cross(B1);
        if (debug)
            std::cout << "B: " << B1 << " K: " << Ktorque << " t: " << torque << std::endl;
    }

    void MagnetForce::GetMFS(const ignition::math::Pose3d &p_self,
                             const ignition::math::Pose3d &p_other,
                             const ignition::math::Vector3d &m_other,
                               ignition::math::Vector3d &mfs
                               )
    {
        //sensor location
        ignition::math::Vector3d p = p_self.Pos() - p_other.Pos();
        ignition::math::Vector3d p_unit = p / p.Length();

        // GEt the field at the sensor location
        double K = 1e-7/pow(p.Length(), 3);
        ignition::math::Vector3d B = K*(3*(m_other.Dot(p_unit))*p_unit - m_other);

        // rotate the B vector into the capsule/body frame
        ignition::math::Vector3d B_body = p_self.Rot().RotateVectorReverse(B);

        //assign vector
        mfs.X() = B_body[0];
        mfs.Y() = B_body[1];
        mfs.Z() = B_body[2];
    }

    GZ_REGISTER_MODEL_PLUGIN(MagnetForce)

} // namespace gazebo