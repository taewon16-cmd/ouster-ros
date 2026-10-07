#pragma once

#include <optional>
#include <string>

#include <ouster/client.h>
#include <ouster/types.h>

namespace ouster_ros {

/// Config passed to configure_sensor() / create_sensor_client().
/// staged is the one-shot from construction or the set_config service.
/// When staged is empty, requested (rebuilt from ROS parameters) is used.
/// The sensor active config is not copied into the payload.
struct OsConfigurePlan {
    ouster::sdk::core::SensorConfig config;
    bool persist{false};
    bool from_staged{false};
    /// True when udp_dest is unset. compose_config_flags maps this to
    /// CONFIG_UDP_DEST_AUTO. An empty string is not auto: it is a value.
    bool udp_dest_auto{false};
};

inline OsConfigurePlan plan_live_sensor_configure(
    const std::optional<ouster::sdk::core::SensorConfig>& staged,
    const ouster::sdk::core::SensorConfig& requested,
    bool persist_param,
    bool persist_already_consumed) {
    OsConfigurePlan plan;
    if (staged.has_value()) {
        plan.config = *staged;
        plan.from_staged = true;
    } else {
        plan.config = requested;
        plan.from_staged = false;
    }
    plan.persist = persist_param && !persist_already_consumed;
    plan.udp_dest_auto = !plan.config.udp_dest.has_value();
    return plan;
}

inline uint8_t live_configure_flags(const OsConfigurePlan& plan, bool force_reinit) {
    uint8_t flags = 0;
    if (plan.udp_dest_auto) {
        flags = static_cast<uint8_t>(flags | ouster::sdk::sensor::CONFIG_UDP_DEST_AUTO);
    }
    if (plan.persist) {
        flags = static_cast<uint8_t>(flags | ouster::sdk::sensor::CONFIG_PERSIST);
    }
    if (force_reinit) {
        flags = static_cast<uint8_t>(flags | ouster::sdk::sensor::CONFIG_FORCE_REINIT);
    }
    return flags;
}

/// Ports the driver would print and bind. Unset optionals stay unset.
/// -1 means the field is not in the payload (client treats that as 0).
struct OsPortSnapshot {
    int lidar{-1};
    int imu{-1};
};

inline OsPortSnapshot snapshot_ports(const ouster::sdk::core::SensorConfig& config) {
    OsPortSnapshot out;
    if (config.udp_port_lidar.has_value()) {
        out.lidar = static_cast<int>(*config.udp_port_lidar);
    }
    if (config.udp_port_imu.has_value()) {
        out.imu = static_cast<int>(*config.udp_port_imu);
    }
    return out;
}

}  // namespace ouster_ros
