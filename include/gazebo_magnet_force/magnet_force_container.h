
#ifndef INCLUDE_MAC_GAZEBO_MAGNET_FORCE_MAGNET_FORCE_CONTAINER_H_
#define INCLUDE_MAC_GAZEBO_MAGNET_FORCE_MAGNET_FORCE_CONTAINER_H_
// library defined
#include <iostream>
#include <vector>
#include <memory>
#include <cstdint>
#include "string"
// gerekli kutuphaneler eklendi
#include <gazebo/common/common.hh>
// plugin icin gerekli kutuphane eklendi

namespace gazebo
{

    class MagnetForceContainer
    {
    public:
        MagnetForceContainer()
        {
        }

        static MagnetForceContainer &Get()
        {
            static MagnetForceContainer instance;
            return instance;
        }

        struct Magnet
        {
            bool calculate;
            ignition::math::Vector3d moment;
            ignition::math::Pose3d offset;
            ignition::math::Pose3d pose;
            std::uint32_t model_id;
        };

        typedef std::shared_ptr<Magnet> MagnetPtr;
        typedef std::vector<MagnetPtr> MagnetPtrV;

        void Add(MagnetPtr mag)
        {//magnet ekleme 
            std::cout << "Adding magnet id:" << mag->model_id << std::endl;
            this->magnets.push_back(mag);
            std::cout << "Total: " << this->magnets.size() << "magnets" << std::endl;
        }

        void Remove(MagnetPtr mag)
        { // magnet silme
            std::cout << "Removing mag id:" << mag->model_id << std::endl;
            this->magnets.erase(std::remove(this->magnets.begin(), this->magnets.end(), mag), this->magnets.end());
            std::cout << "Total: " << this->magnets.size() << "magnets" << std::endl;
        }

        MagnetPtrV magnets;
    };

} // namespace gazebo

#endif