#pragma once
#include <cstdint>
#include <cmath>

enum class CommandType : uint8_t {
    GO_TO,
    DEPTH,
    TURN,
    YUNUSLAMA
};

//Singleton
struct VehicleContext {
    volatile float current_dist_x = 0.0f;
    volatile float current_yaw    = 0.0f;
    volatile float current_depth  = 0.0f;

    float target_speed = 0.0f;
    float target_yaw   = 0.0f;
    float target_depth = 0.0f;
    float target_pitch = 0.0f;
    bool pitch_override = false;

    void resetLegPosition(); 
};

class MissionCommand {
public:
    virtual ~MissionCommand() = default;
    virtual void onStart(VehicleContext& ctx) = 0;
    virtual void onUpdate(VehicleContext& ctx) {}
    virtual bool isFinished(const VehicleContext& ctx) const = 0;
    virtual void onFinish(VehicleContext& ctx) {}
    virtual bool isParallel() const { return false; } // Varsayılan olarak bloklayan komut
};