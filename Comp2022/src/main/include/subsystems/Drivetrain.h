// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#pragma once

#include "Constants.h"
#include "Vision.h"

#include <frc/Encoder.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/Trajectory.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "ctre/Phoenix.h"

#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/SubsystemBase.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

using namespace std;
using namespace units::acceleration;
using namespace units::angle;
using namespace units::angular_acceleration;
using namespace units::angular_velocity;
using namespace units::length;
using namespace units::velocity;
using namespace units::voltage;

/**
 *
 *
 * @author ExampleAuthor
 */
class Drivetrain : public frc2::SubsystemBase
{
private:
    // It's desirable that everything possible is private except
    // for methods that implement subsystem capabilities

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    WPI_TalonFX m_motorL1{ 1 }; //4,R4
    WPI_TalonFX m_motorR3{ 3 }; //2,L2
    frc::DifferentialDrive m_diffDrive{ m_motorL1, m_motorR3 };
    WPI_TalonFX m_motorL2{ 2 }; //3,R3
    WPI_TalonFX m_motorR4{ 4 }; //1,L1

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // Gyro - same on robot and in simulation
    PigeonIMU m_gyro{ 0 };

    //    Declare constants
    const int m_driveDebug = 0; // Debug flag to disable extra logging calls
    const int kPidIndex = 0;    // PID slot index for sensors
    const int kCANTimeout = 10; // CAN timeout in msec to wait for response

    // TODO: adjust kV and kA angular from robot characterization
    frc::sim::DifferentialDrivetrainSim m_driveSim{ frc::LinearSystemId::IdentifyDrivetrainSystem(
                                                        DriveConstants::kv,
                                                        DriveConstants::ka,
                                                        DriveConstants::KvAngular,
                                                        DriveConstants::KaAngular,
                                                        DriveConstants::kTrackWidthMeters),
                                                    DriveConstants::kTrackWidthMeters,
                                                    frc::DCMotor::Falcon500(2),
                                                    DriveConstants::kGearRatio,
                                                    DriveConstants::kWheelDiaMeters / 2 };
    // Declare module variables
    bool m_talonValidL1; // Health indicator for drive Talon Left 1
    bool m_talonValidL2; // Health indicator for drive Talon Left 2
    bool m_talonValidR3; // Health indicator for drive Talon Right 3
    bool m_talonValidR4; // Health indicator for drive Talon Right 4
    bool m_pigeonValid;  // Health indicator for Pigeon IMU

    // Joysticks
    double m_driveXScaling;  // Scaling applied to Joystick
    double m_driveYScaling;  // Scaling applied to Joystick
    double m_driveQTScaling; // Scaling applied to Joystick when QuickTurn enabled
    bool m_throttleZeroed;   // Throttle joystick zeroed check for safety

    // Talon input filter settings
    double m_openLoopRampRate;
    double m_closedLoopRampRate;

    // Drive modes
    bool m_brakeMode;   // Brake or Coast Mode for Talons
    bool m_isQuickTurn; // Setting for quickturn in curvature drive

    // Odometry and telemetry
    meter_t m_distanceLeft;
    meter_t m_distanceRight;
    frc::DifferentialDriveWheelSpeeds m_wheelSpeeds;
    frc::DifferentialDriveOdometry m_odometry{ GetHeadingAngle() };
    frc::Field2d m_field;

    double m_currentl1 = 0.0; // Motor L1 output current from Falcon
    double m_currentL2 = 0.0; // Motor L2 output current from Falcon
    double m_currentR3 = 0.0; // Motor R3 output current from Falcon
    double m_currentR4 = 0.0; // Motor R4 output current from Falcon

    // limelight drive
    double m_turnPidKp = 0.1;
    double m_turnPidKi = 0.0;
    double m_turnPidKd = 0.0;
    double m_throttlePidKp = 0.1;
    double m_throttlePidKi = 0.0;
    double m_throttlePidKd = 0.0;
    double m_maxTurn;
    double m_maxThrottle;
    double m_targetAngle;
    double m_targetDistance;
    double m_angleThreshold;
    double m_distThreshold;
    double m_throttleShape;
    double m_vertOffset1;
    double m_vertOffset2;
    double m_distance1;
    double m_distance2;
    double m_slope;
    double m_distOffset;
    double m_limelightDistance;

    // Ramsete path follower drive
    double m_ramsetePidKp = 0.0;
    double m_ramsetePidKi = 0.0;
    double m_ramsetePidKd = 0.0;
    double m_ramseteB = 0.0;
    double m_ramseteZeta = 0.0;

    // Current limit settings
    SupplyCurrentLimitConfiguration m_supplyCurrentLimits = { true, 45.0, 45.0, 0.001 };
    StatorCurrentLimitConfiguration m_statorCurrentLimits = { true, 80.0, 80.0, 0.001 };

    // DriveWithLimelight pid controller objects
    frc2::PIDController m_turnPid{ 0.0, 0.0, 0.0 };
    frc2::PIDController m_throttlePid{ 0.0, 0.0, 0.0 };

    // Ramsete follower objects
    frc::Trajectory m_trajectory;
    frc::RamseteController m_ramseteController;
    frc::SimpleMotorFeedforward<meter> m_feedforward{ DriveConstants::ks, DriveConstants::kv, DriveConstants::ka };
    frc::DifferentialDriveKinematics m_kinematics{ DriveConstants::kTrackWidthMeters };
    frc2::PIDController m_leftPid{ 0.0, 0.0, 0.0 };
    frc2::PIDController m_rightPid{ 0.0, 0.0, 0.0 };
    frc::Timer m_trajTimer;

    // Path following variables
    double m_tolerance;

    ///////////////////////////////////////////////////////////////////////////

    // Initialization methods
    void ConfigFileLoad(void);
    void TalonMasterInitialize(WPI_TalonFX &motor, bool inverted);
    void TalonFollowerInitialize(WPI_TalonFX &motor, int master);

    // Periodic update methods
    void UpdateOdometry(void);
    void UpdateDashboardValues(void);

    // Encoders
    void ResetEncoders(void);
    meter_t GetDistanceMetersLeft(void);
    meter_t GetDistanceMetersRight(void);
    frc::DifferentialDriveWheelSpeeds GetWheelSpeedsMPS(void);

    int DistanceToNativeUnits(units::meter_t position);
    int VelocityToNativeUnits(units::meters_per_second_t velocity);

    void ResetGyro(void);
    degree_t GetHeadingAngle(void);

    void ResetOdometry(frc::Pose2d pose);

    void PlotTrajectory(frc::Trajectory trajectory);

public:
    Drivetrain();

    void Periodic() override;
    void SimulationPeriodic() override;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    void Initialize(void);
    void FaultDump(void);

    void ResetSensors(void);
    void SetBrakeMode(bool brakeMode);
    void MoveSetQuickTurn(bool quickTurn);
    void MoveStop(void);
    bool MoveIsStopped(void);

    void TankDriveVolts(volt_t left, volt_t right);

    // Teleop mode
    void MoveWithJoysticksInit(void);
    void MoveWithJoysticks(frc::XboxController *driverPad);
    void MoveWithJoysticksEnd(void);

    void MoveWithLimelightInit(bool m_endAtTarget);
    double GetLimelightDistance(void);
    void MoveWithLimelightExecute(double tx, double ty, bool tv);
    bool MoveWithLimelightIsFinished(double tx, bool tv);
    void MoveWithLimelightEnd();

    // Autonomous - Ramsete follower command
    void RamseteFollowerInit(string pathName, bool resetOdometry);
    void RamseteFollowerExecute(void);
    bool RamseteFollowerIsFinished(void);
    void RamseteFollowerEnd(void);

    void DriveBackward(double tx, double ty, bool tv);
};
