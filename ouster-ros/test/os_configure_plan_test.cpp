#include <gtest/gtest.h>

#include <optional>
#include <string>

#include <ouster/client.h>
#include <ouster/types.h>

#include "ouster_ros/os_configure_plan.hpp"
#include "../src/point_cloud_processor_factory.h"

using ouster::sdk::core::LidarMode;
using ouster::sdk::core::SensorConfig;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::UDPProfileLidar;
using ouster::sdk::core::default_sensor_info;
using ouster::sdk::core::lidar_mode_of_string;
using ouster::sdk::sensor::CONFIG_PERSIST;
using ouster::sdk::sensor::CONFIG_UDP_DEST_AUTO;
using ouster_ros::OsConfigurePlan;
using ouster_ros::live_configure_flags;
using ouster_ros::plan_live_sensor_configure;
using ouster_ros::snapshot_ports;

namespace {

SensorConfig requested_lidar2() {
    SensorConfig config;
    config.udp_dest = "10.0.11.100";
    config.udp_port_lidar = 7503;
    config.udp_port_imu = 7504;
    config.udp_profile_lidar = UDPProfileLidar::RNG19_RFL8_SIG16_NIR16;
    config.lidar_mode = lidar_mode_of_string("1024x10");
    return config;
}

SensorConfig active_during_failed_reconnect() {
    SensorConfig config;
    config.udp_dest = "";
    config.udp_port_lidar = 7502;
    config.udp_port_imu = 7503;
    config.udp_profile_lidar = UDPProfileLidar::OFF;
    config.lidar_mode = lidar_mode_of_string("1024x10");
    return config;
}

OsConfigurePlan reconnect(const SensorConfig& requested, bool persist_param,
                          bool persist_already_consumed) {
    return plan_live_sensor_configure(std::nullopt, requested, persist_param,
                                      persist_already_consumed);
}

}  // namespace

TEST(OsConfigurePlan, ExplicitUdpDestSurvivesEmptyActiveDest) {
    const auto active = active_during_failed_reconnect();
    const auto plan = reconnect(requested_lidar2(), false, true);
    EXPECT_FALSE(plan.from_staged);
    EXPECT_EQ(plan.config.udp_dest.value_or(""), "10.0.11.100");
    EXPECT_NE(plan.config.udp_dest.value_or(""), active.udp_dest.value_or(""));
    EXPECT_FALSE(plan.udp_dest_auto);
    EXPECT_EQ(live_configure_flags(plan, false) & CONFIG_UDP_DEST_AUTO, 0);
}

TEST(OsConfigurePlan, ExplicitProfileSurvivesOff) {
    const auto plan = reconnect(requested_lidar2(), false, true);
    ASSERT_TRUE(plan.config.udp_profile_lidar.has_value());
    EXPECT_EQ(*plan.config.udp_profile_lidar, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16);
    EXPECT_NE(*plan.config.udp_profile_lidar, UDPProfileLidar::OFF);
}

TEST(OsConfigurePlan, ExplicitPortsAreThePayloadNotTheActiveConfig) {
    const auto active = active_during_failed_reconnect();
    const auto requested = requested_lidar2();
    const auto plan = reconnect(requested, false, true);
    const auto active_ports = snapshot_ports(active);
    const auto selected = snapshot_ports(plan.config);
    const auto payload = snapshot_ports(plan.config);
    const auto client = snapshot_ports(plan.config);
    EXPECT_EQ(active_ports.lidar, 7502);
    EXPECT_EQ(active_ports.imu, 7503);
    EXPECT_EQ(selected.lidar, 7503);
    EXPECT_EQ(selected.imu, 7504);
    EXPECT_EQ(payload.lidar, selected.lidar);
    EXPECT_EQ(payload.imu, selected.imu);
    EXPECT_EQ(client.lidar, selected.lidar);
    EXPECT_EQ(client.imu, selected.imu);
    EXPECT_NE(selected.lidar, active_ports.lidar);
}

TEST(OsConfigurePlan, AutoUdpWhenDestUnset) {
    SensorConfig requested;
    requested.udp_port_lidar = 7503;
    const auto plan = reconnect(requested, false, true);
    EXPECT_FALSE(plan.config.udp_dest.has_value());
    EXPECT_TRUE(plan.udp_dest_auto);
    EXPECT_NE(live_configure_flags(plan, false) & CONFIG_UDP_DEST_AUTO, 0);
}

TEST(OsConfigurePlan, UnspecifiedPortsStayUnset) {
    SensorConfig requested;
    requested.udp_dest = "10.0.11.100";
    requested.udp_profile_lidar = UDPProfileLidar::RNG19_RFL8_SIG16_NIR16;
    const auto plan = reconnect(requested, false, true);
    EXPECT_FALSE(plan.config.udp_port_lidar.has_value());
    EXPECT_FALSE(plan.config.udp_port_imu.has_value());
    const auto ports = snapshot_ports(plan.config);
    EXPECT_EQ(ports.lidar, -1);
    EXPECT_EQ(ports.imu, -1);
}

TEST(OsConfigurePlan, LidarModeIsReapplied) {
    const auto plan = reconnect(requested_lidar2(), false, true);
    ASSERT_TRUE(plan.config.lidar_mode.has_value());
    EXPECT_EQ(*plan.config.lidar_mode, lidar_mode_of_string("1024x10"));
}

TEST(OsConfigurePlan, PersistIsOneShot) {
    const auto first = reconnect(requested_lidar2(), true, false);
    EXPECT_TRUE(first.persist);
    EXPECT_NE(live_configure_flags(first, false) & CONFIG_PERSIST, 0);

    const auto again = reconnect(requested_lidar2(), true, true);
    EXPECT_FALSE(again.persist);
    EXPECT_EQ(live_configure_flags(again, false) & CONFIG_PERSIST, 0);

    const auto never = reconnect(requested_lidar2(), false, false);
    EXPECT_FALSE(never.persist);
}

TEST(OsConfigurePlan, FirstConfigureUsesStagedConfig) {
    SensorConfig staged = requested_lidar2();
    SensorConfig other;
    other.udp_dest = "10.0.11.50";
    other.udp_port_lidar = 7502;
    other.udp_port_imu = 7503;
    other.udp_profile_lidar = UDPProfileLidar::OFF;
    const auto plan = plan_live_sensor_configure(staged, other, false, false);
    EXPECT_TRUE(plan.from_staged);
    EXPECT_EQ(plan.config.udp_dest.value_or(""), "10.0.11.100");
    EXPECT_EQ(snapshot_ports(plan.config).lidar, 7503);
    EXPECT_EQ(snapshot_ports(plan.config).imu, 7504);
    EXPECT_EQ(*plan.config.udp_profile_lidar, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16);
}

TEST(OsConfigurePlan, MetadataKeepsExplicitProfileWhenDestIsSet) {
    const auto mode = lidar_mode_of_string("1024x10");
    ASSERT_TRUE(mode.has_value());
    auto info = default_sensor_info(*mode);
    info.config = requested_lidar2();
    info.format.udp_profile_lidar = UDPProfileLidar::RNG19_RFL8_SIG16_NIR16;
    info.config.lidar_mode = mode;

    const SensorInfo parsed(info.to_json_string());
    EXPECT_EQ(parsed.format.udp_profile_lidar, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16);
    EXPECT_EQ(parsed.config.udp_dest.value_or(""), "10.0.11.100");

    auto cleared = info;
    cleared.config.udp_dest = "";
    const SensorInfo forced_off(cleared.to_json_string());
    EXPECT_EQ(forced_off.format.udp_profile_lidar, UDPProfileLidar::OFF);

    EXPECT_NO_THROW(
        ouster_ros::PointCloudProcessorFactory::create_point_cloud_processor(
            "native", parsed, "os_sensor", false, false, false,
            0, 100000, 1, "", {}));
}
