#ifndef LIDAR_POINTCLOUD_SCAN_PCA9685
#define LIDAR_POINTCLOUD_SCAN_PCA9685
#include <memory>
#include <mutex>
#include "PCA9685/PCA9685.h"

constexpr uint32_t I2C_BUS = 1;
constexpr uint32_t I2C_ADDR_CONTROLLER = 0x40;
constexpr uint32_t PWM_FREQUENCY = 150;

class PCA9685Manager {
private:
    static std::shared_ptr<PCA9685> pcaInstance;
    static std::mutex mutex;
    
    // Private constructor to avoid new managers
    PCA9685Manager() = default;

public:
    // Avoid copy operations
    PCA9685Manager(const PCA9685Manager&) = delete;
    PCA9685Manager& operator=(const PCA9685Manager&) = delete;

    static std::shared_ptr<PCA9685> getInstance() {
        // Protect multiple instantiations
        std::lock_guard<std::mutex> lock(mutex);
        if (!pcaInstance) {
            pcaInstance = std::make_shared<PCA9685>(I2C_BUS, I2C_ADDR_CONTROLLER);
            
            pcaInstance->setPWMFreq(PWM_FREQUENCY);
        }
        return pcaInstance;
    }
};

// PCA9685Manager static members definition
std::shared_ptr<PCA9685> PCA9685Manager::pcaInstance = nullptr;
std::mutex PCA9685Manager::mutex;

#endif //LIDAR_POINTCLOUD_SCAN_PCA9685