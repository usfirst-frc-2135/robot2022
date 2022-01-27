// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/numbers>

using namespace units::acceleration;
using namespace units::length;
using namespace units::velocity;

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace AutoConstants
{
} // namespace AutoConstants

namespace DriveConstants
{
    static constexpr int kLeftEncoderPorts[]{ 1, 2 };
    static constexpr int kRightEncoderPorts[]{ 3, 4 };

    // Odometry constants
    static constexpr int kEncoderCPR = 2048; // CPR is 2048 for new TalonFX
// Gear Ratio Checker
#if 1
    static constexpr double kGearRatio = 12.75;        // grogu- motor rotations to output shaft
    static constexpr meter_t kWheelDiaMeters = 6.0_in; // Units library does the conversion
#else
    static constexpr double kGearRatio = 8.45;         // comp2022- motor rotations to output shaft
    static constexpr meter_t kWheelDiaMeters = 4.0_in; // Units library does the conversion
#endif
    //static constexpr double kGearRatio = 12.75;
    static constexpr meter_t kEncoderMetersPerCount =
        (kWheelDiaMeters * wpi::numbers::pi) / static_cast<double>(kEncoderCPR) / kGearRatio;
    static constexpr meter_t kTrackWidthMeters = 0.6477_m; // Measured track width
                                                           // Gear reduction

    // Kinematics values for 2135 Bebula - 2019 B-bot
    static constexpr auto ks = 0.65_V;
    static constexpr auto kv = 2.84_V / 1_mps;
    static constexpr auto ka = 0.309_V / 1_mps_sq;
    static constexpr auto KvAngular = 1.5_V / 1_rad_per_s;
    static constexpr auto KaAngular = 0.3_V / 1_rad_per_s_sq;

    static constexpr meters_per_second_t kMaxSpeed = 1.1336_mps;
    static constexpr meters_per_second_squared_t kMaxAcceleration = 10.668_mps_sq;

} // namespace DriveConstants

namespace IntakeConstants
{
} // namespace IntakeConstants

namespace ConveyorConstants
{
} // namespace ConveyorConstants

namespace ClimberConstants
{
    static constexpr int kClimberEncoderCPR = 2048;
    static constexpr double kClimberRolloutRatio = 0.1357; // inches per falcon rotation
    static constexpr double kCircumInches = 0.428 * M_PI;
    static constexpr double kInchesPerCount =
        kCircumInches * kClimberRolloutRatio * (1.0 / (double)kClimberEncoderCPR);

} // namespace ClimberConstants

namespace ShooterConstants
{
} // namespace ShooterConstants

namespace OIConstants
{
} // namespace OIConstants
