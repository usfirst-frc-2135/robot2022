// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "RobotContainer.h"
#include "frc2135/RobotConfig.h"
#include "frc2135/TalonUtils.h"

#include <frc/Filesystem.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/RobotState.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <fstream>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
#include <wpi/SmallString.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "subsystems/Drivetrain.h"

#include <frc/smartdashboard/SmartDashboard.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

Drivetrain::Drivetrain()
{
    SetName("Drivetrain");
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SetSubsystem("Drivetrain");

    AddChild("Diff Drive", &m_diffDrive);
    m_diffDrive.SetSafetyEnabled(true);
    m_diffDrive.SetExpiration(0.1_s);
    m_diffDrive.SetMaxOutput(1.0);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    //  Validate Talon controllers, reset and display firmware versions
    m_talonValidL1 = frc2135::TalonUtils::TalonCheck(m_motorL1, "DT", "L1");
    m_talonValidL2 = frc2135::TalonUtils::TalonCheck(m_motorL2, "DT", "L2");
    m_talonValidR3 = frc2135::TalonUtils::TalonCheck(m_motorR3, "DT", "R3");
    m_talonValidR4 = frc2135::TalonUtils::TalonCheck(m_motorR4, "DT", "R4");
    m_pigeonValid = frc2135::TalonUtils::PigeonIMUInitialize(m_gyro);

    //  Load config file values
    ConfigFileLoad();

    //  Initialize Talon motor controllers
    if (m_talonValidL1)
        TalonMasterInitialize(m_motorL1, false);
    if (m_talonValidL2)
        TalonFollowerInitialize(m_motorL2, 1);
    if (m_talonValidR3)
        TalonMasterInitialize(m_motorR3, true);
    if (m_talonValidR4)
        TalonFollowerInitialize(m_motorR4, 3);

    // If either master drive talons are valid, enable safety timer
    m_diffDrive.SetSafetyEnabled(m_talonValidL1 || m_talonValidR3);

    // Set up Field 2d for simulator
    frc::SmartDashboard::PutData("Field", &m_field);

    // Limelight Pid Controllers
    m_turnPid = frc2::PIDController(m_turnPidKp, m_turnPidKi, m_turnPidKd);
    m_throttlePid = frc2::PIDController(m_throttlePidKp, m_throttlePidKi, m_throttlePidKd);

    // Ramsete Pid Controllers
    m_leftPid = frc2::PIDController(m_ramsetePidKp, m_ramsetePidKi, m_ramsetePidKd);
    m_rightPid = frc2::PIDController(m_ramsetePidKp, m_ramsetePidKi, m_ramsetePidKd);

    m_ramseteController = frc::RamseteController(m_ramseteB, m_ramseteZeta);

    Initialize();
}

void Drivetrain::SimulationInit()
{
    m_pigeonValid = true;
}

void Drivetrain::Periodic()
{
    // Put code here to be run every loop
    UpdateOdometry();
    UpdateDashboardValues();
    m_field.SetRobotPose(GetPose());
}

void Drivetrain::SimulationPeriodic()
{
    // This method will be called once per scheduler run when in simulation

    TalonFXSimCollection &leftSim(m_motorL1.GetSimCollection());
    TalonFXSimCollection &rightSim(m_motorR3.GetSimCollection());
    BasePigeonSimCollection &pidgeonSim(m_gyro.GetSimCollection());

    /* Pass the robot battery voltage to the simulated Talon FXs */
    leftSim.SetBusVoltage(frc::RobotController::GetInputVoltage());
    rightSim.SetBusVoltage(frc::RobotController::GetInputVoltage());

    /*
	 * CTRE simulation is low-level, so SimCollection inputs and outputs are not 
     * affected by SetInverted(). Only the regular user-level API calls are affected.
	 *
	 * WPILib expects +V to be forward. Positive motor output lead voltage is ccw. We
	 * observe on our physical robot that this is reverse for the right motor, so negate it.
	 *
	 * We are hard-coding the negation of the values instead of using GetInverted() so we can
     * catch a possible bug in the robot code where the wrong value is passed to SetInverted().
	 */
    m_driveSim.SetInputs(leftSim.GetMotorOutputLeadVoltage() * 1_V, -rightSim.GetMotorOutputLeadVoltage() * 1_V);

    /* Advance the model by 20 ms. */
    m_driveSim.Update(20_ms);

    /*
	 * Update all of our sensors.
	 *
	 * Since WPILib's simulation class is assuming +V is forward, but -V is forward for the 
     * right motor, we need to negate the position reported by the simulation class. Basically, 
     * we negated the input, so we need to negate the output.
	 *
	 * We also observe on our physical robot that a positive voltage across the output leads 
     * results in a negative sensor velocity for both the left and right motors, so we need to 
     * negate the output once more.
	 * Left output: +1 * -1 = -1
	 * Right output: -1 * -1 = +1
	 */
    leftSim.SetIntegratedSensorRawPosition(MetersToNativeUnits(m_driveSim.GetLeftPosition()));
    leftSim.SetIntegratedSensorVelocity(MPSToNativeUnits(m_driveSim.GetLeftVelocity()));
    rightSim.SetIntegratedSensorRawPosition(MetersToNativeUnits(-m_driveSim.GetRightPosition()));
    rightSim.SetIntegratedSensorVelocity(MPSToNativeUnits(-m_driveSim.GetRightVelocity()));

    pidgeonSim.SetRawHeading(m_driveSim.GetHeading().Degrees().to<double>());
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// Put methods for controlling this subsystem
// here. Call these from Commands.

void Drivetrain::Initialize(void)
{
    spdlog::info("DT Initialize");

    // When disabled, set low gear and coast mode to allow easier pushing
    m_brakeMode = false;
    m_throttleZeroed = false;
    MoveSetQuickTurn(false);

    SetBrakeMode(m_brakeMode);
    MoveStop();

    // Initialize the odometry
    ResetOdometry({ { 0_m, 0_m }, 0_deg });
    m_driveSim.SetPose(m_odometry.GetPose());
    m_field.SetRobotPose(m_odometry.GetPose());

    // Initialize PID values for velocity control
    SyncTalonPIDFromDashboard();
}

void Drivetrain::FaultDump(void)
{
    // Dump all Talon faults
    frc2135::TalonUtils::TalonFaultDump("DT L1", m_motorL1);
    frc2135::TalonUtils::TalonFaultDump("DT L2", m_motorL2);
    frc2135::TalonUtils::TalonFaultDump("DT R3", m_motorR3);
    frc2135::TalonUtils::TalonFaultDump("DT R4", m_motorR4);
}

///////////////////////////////////////////////////////////////////////////////
//
//  Initialization helper methods
//
void Drivetrain::ConfigFileLoad(void)
{
    //  Retrieve drivetrain modified parameters from RobotConfig
    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();
    config->GetValueAsDouble("DT_DriveXScaling", m_driveXScaling, 0.75);
    config->GetValueAsDouble("DT_DriveYScaling", m_driveYScaling, 0.75);
    config->GetValueAsDouble("DT_QuickTurnScaling", m_driveQTScaling, 0.5);
    config->GetValueAsDouble("DT_OpenLoopRampRate", m_openLoopRampRate, 0.5);
    config->GetValueAsDouble("DT_ClosedLoopRampRate", m_closedLoopRampRate, 0.0);
    config->GetValueAsDouble("DT_StoppedTolerance", m_tolerance, 0.05);

    // retrieve limelight values from config file and put on smartdashboard
    config->GetValueAsDouble("DTL_TurnPidKp", m_turnPidKp, 0.045);
    config->GetValueAsDouble("DTL_TurnPidKi", m_turnPidKi, 0.0);
    config->GetValueAsDouble("DTL_TurnPidKd", m_turnPidKd, 0.0);
    config->GetValueAsDouble("DTL_ThrottlePidKp", m_throttlePidKp, 0.02);
    config->GetValueAsDouble("DTL_ThrottlePidKi", m_throttlePidKi, 0.0);
    config->GetValueAsDouble("DTL_ThrottlePidKd", m_throttlePidKd, 0.0);
    config->GetValueAsDouble("DTL_MaxTurn", m_maxTurn, 0.4);
    config->GetValueAsDouble("DTL_MaxThrottle", m_maxThrottle, 0.2);
    config->GetValueAsDouble("DTL_ThrottleShape", m_throttleShape, 10.0);
    config->GetValueAsDouble("DTL_TargetAngle", m_targetAngle, 0.0);
    config->GetValueAsDouble("DTL_TargetDistance", m_targetDistance, 12.0);
    config->GetValueAsDouble("DTL_AngleThreshold", m_angleThreshold, 3.0);
    config->GetValueAsDouble("DTL_DistThreshold", m_distThreshold, 6.0);
    config->GetValueAsDouble("DTL_Distance1", m_distance1, 0.0);
    config->GetValueAsDouble("DTL_Distance2", m_distance2, 0.0);
    config->GetValueAsDouble("DTL_VertOffset1", m_vertOffset1, 0.0);
    config->GetValueAsDouble("DTL_VertOffset2", m_vertOffset2, 0.0);

    // Ramsete follower settings
    config->GetValueAsDouble("DTR_RamsetePidKf", m_ramsetePidKf, 0.0);
    config->GetValueAsDouble("DTR_RamsetePidKp", m_ramsetePidKp, 2.0);
    config->GetValueAsDouble("DTR_RamsetePidKi", m_ramsetePidKi, 0.0);
    config->GetValueAsDouble("DTR_RamsetePidKd", m_ramsetePidKd, 0.0);
    config->GetValueAsDouble("DTR_RamseteB", m_ramseteB, 2.0);
    config->GetValueAsDouble("DTR_RamseteZeta", m_ramseteZeta, 0.7);

    // Put tunable items to dashboard
    frc::SmartDashboard::PutNumber("DT_Tolerance", m_tolerance);

    frc::SmartDashboard::PutNumber("DTL_TurnPidKp", m_turnPidKp);
    frc::SmartDashboard::PutNumber("DTL_TurnPidKi", m_turnPidKi);
    frc::SmartDashboard::PutNumber("DTL_TurnPidKd", m_turnPidKd);
    frc::SmartDashboard::PutNumber("DTL_ThrottlePidKp", m_throttlePidKp);
    frc::SmartDashboard::PutNumber("DTL_ThrottlePidKi", m_throttlePidKi);
    frc::SmartDashboard::PutNumber("DTL_ThrottlePidKd", m_throttlePidKd);
    frc::SmartDashboard::PutNumber("DTL_MaxTurn", m_maxTurn);
    frc::SmartDashboard::PutNumber("DTL_MaxThrottle", m_maxThrottle);
    frc::SmartDashboard::PutNumber("DTL_ThrottleShape", m_throttleShape);
    frc::SmartDashboard::PutNumber("DTL_TargetAngle", m_targetAngle);
    frc::SmartDashboard::PutNumber("DTL_TargetDistance", m_targetDistance);
    frc::SmartDashboard::PutNumber("DTL_AngleThreshold", m_angleThreshold);
    frc::SmartDashboard::PutNumber("DTL_DistThreshold", m_distThreshold);
    frc::SmartDashboard::PutNumber("DTL_Distance1", m_distance1);
    frc::SmartDashboard::PutNumber("DTL_Distance2", m_distance2);
    frc::SmartDashboard::PutNumber("DTL_VertOffset1", m_vertOffset1);
    frc::SmartDashboard::PutNumber("DTL_VertOffset2", m_vertOffset2);

    frc::SmartDashboard::PutNumber("DTR_ramsetePidKf", m_ramsetePidKf);
    frc::SmartDashboard::PutNumber("DTR_ramsetePidKp", m_ramsetePidKp);
    frc::SmartDashboard::PutNumber("DTR_ramsetePidKi", m_ramsetePidKi);
    frc::SmartDashboard::PutNumber("DTR_ramsetePidKd", m_ramsetePidKd);
    frc::SmartDashboard::PutNumber("DTR_ramseteB", m_ramseteB);
    frc::SmartDashboard::PutNumber("DTR_ramseteZeta", m_ramseteZeta);
}

void Drivetrain::TalonMasterInitialize(WPI_TalonFX &motor, bool inverted)
{
    //  Setup motor direction, neutral mode, voltage compensation, and encoder
    motor.SetInverted(inverted);
    motor.SetNeutralMode(NeutralMode::Coast);

    motor.Set(ControlMode::PercentOutput, 0.0);
    motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, kPidIndex, kCANTimeout);
    motor.SetSensorPhase(false);
    motor.SetSelectedSensorPosition(0, kPidIndex, kCANTimeout);

    motor.ConfigOpenloopRamp(m_openLoopRampRate, kCANTimeout);
    motor.ConfigClosedloopRamp(m_closedLoopRampRate, kCANTimeout);

    motor.ConfigSupplyCurrentLimit(m_supplyCurrentLimits);
    motor.ConfigStatorCurrentLimit(m_statorCurrentLimits);
}

void Drivetrain::TalonFollowerInitialize(WPI_TalonFX &motor, int master)
{
    motor.Set(ControlMode::Follower, master);
    motor.SetInverted(InvertType::FollowMaster);
    motor.SetNeutralMode(NeutralMode::Coast);

    motor.ConfigSupplyCurrentLimit(m_supplyCurrentLimits);
    motor.ConfigStatorCurrentLimit(m_statorCurrentLimits);
}

///////////////////////////////////////////////////////////////////////////////
//
//  Periodic helper methods
//
void Drivetrain::UpdateOdometry(void)
{
    // Get all sensors and update odometry
    m_distanceLeft = GetDistanceMetersLeft();
    m_distanceRight = GetDistanceMetersRight();
    m_wheelSpeeds = GetWheelSpeedsMPS();
    m_odometry.Update(frc::Rotation2d(GetHeadingAngle()), m_distanceLeft, m_distanceRight);

    if (m_driveDebug)
    {
        if (m_talonValidL1)
            m_currentl1 = m_motorL1.GetOutputCurrent();
        if (m_talonValidL2)
            m_currentL2 = m_motorL2.GetOutputCurrent();
        if (m_talonValidR3)
            m_currentR3 = m_motorR3.GetOutputCurrent();
        if (m_talonValidR4)
            m_currentR4 = m_motorR4.GetOutputCurrent();
    }
}

void Drivetrain::UpdateDashboardValues(void)
{
    static int periodicInterval = 0;

    frc::SmartDashboard::PutNumber("DT_distanceLeft", m_distanceLeft.to<double>());
    frc::SmartDashboard::PutNumber("DT_distanceRight", m_distanceRight.to<double>());
    frc::SmartDashboard::PutNumber("DT_wheelSpeedLeft", m_wheelSpeeds.left.to<double>());
    frc::SmartDashboard::PutNumber("DT_wheelSpeedRight", m_wheelSpeeds.right.to<double>());
    frc::SmartDashboard::PutNumber("DT_getHeadingAngle", GetHeadingAngle().to<double>());
    frc::SmartDashboard::PutNumber("DT_heading", GetPose().Rotation().Degrees().to<double>());
    frc::SmartDashboard::PutNumber("DT_currentX", GetPose().X().to<double>());
    frc::SmartDashboard::PutNumber("DT_currentY", GetPose().Y().to<double>());

    frc::SmartDashboard::PutNumber("DT_Current_L1", m_currentl1);
    frc::SmartDashboard::PutNumber("DT_Current_L2", m_currentL2);
    frc::SmartDashboard::PutNumber("DT_Current_R3", m_currentR3);
    frc::SmartDashboard::PutNumber("DT_Current_R4", m_currentR4);

    // Only update indicators every 100 ms to cut down on network traffic
    if ((periodicInterval++ % 5 == 0) && (m_driveDebug > 1))
    {
        spdlog::info(
            "DT deg {} LR dist {} {} amps {:.1f} {:.1f} {:.1f} {:.1f}",
            GetPose().Rotation().Degrees(),
            m_distanceLeft,
            m_distanceRight,
            m_currentl1,
            m_currentL2,
            m_currentR3,
            m_currentR4);
    }
}

///////////////////////////////////////////////////////////////////////////////
//
//  Getters/Setters
//
//  Wheel encoders
//
void Drivetrain::ResetEncoders()
{
    if (m_talonValidL1)
        m_motorL1.SetSelectedSensorPosition(0);
    if (m_talonValidR3)
        m_motorR3.SetSelectedSensorPosition(0);
}

meter_t Drivetrain::GetDistanceMetersLeft()
{
    if (m_talonValidL1)
        return DriveConstants::kEncoderMetersPerCount * m_motorL1.GetSelectedSensorPosition(kPidIndex);

    return 0_m;
}

meter_t Drivetrain::GetDistanceMetersRight()
{
    if (m_talonValidR3)
        return DriveConstants::kEncoderMetersPerCount * m_motorR3.GetSelectedSensorPosition(kPidIndex);

    return 0_m;
}

frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeedsMPS()
{
    meters_per_second_t leftVelocity = 0_mps;
    meters_per_second_t rightVelocity = 0_mps;

    if (m_talonValidL1)
        leftVelocity = DriveConstants::kEncoderMetersPerCount * m_motorL1.GetSelectedSensorVelocity() * 10 / 1_s;
    if (m_talonValidR3)
        rightVelocity = DriveConstants::kEncoderMetersPerCount * m_motorR3.GetSelectedSensorVelocity() * 10 / 1_s;

    return { leftVelocity, rightVelocity };
}

// Helper methods to convert between meters and native units
int Drivetrain::MetersToNativeUnits(units::meter_t position)
{
    return position / DriveConstants::kEncoderMetersPerCount;
}

int Drivetrain::MPSToNativeUnits(units::meters_per_second_t velocity)
{
    return (velocity / DriveConstants::kEncoderMetersPerCount / 10).to<double>();
}

double Drivetrain::JoystickOutputToNative(double output)
{
    // Phoenix native encoder units are CPR / 100 msec
    double outputScaling = 1.0;
    return (output * outputScaling * DriveConstants::kRPM * DriveConstants::kEncoderCPR) / (60.0 * 10.0);
}

void Drivetrain::VelocityArcadeDrive(double yOutput, double xOutput)
{
    double leftOutput = JoystickOutputToNative(std::clamp(yOutput + xOutput, -1.0, 1.0));
    double rightOutput = JoystickOutputToNative(std::clamp(yOutput - xOutput, -1.0, 1.0));

    m_motorL1.Set(ControlMode::Velocity, leftOutput);
    m_motorR3.Set(ControlMode::Velocity, rightOutput);

    frc::SmartDashboard::PutNumber("VCL_LeftOutput", leftOutput);
    frc::SmartDashboard::PutNumber("VCL_RightOuput", rightOutput);
    // spdlog::info("DT motor speeds - left {:.1f} right {:.1f}", leftOutput, rightOutput);

    double curLeftOutput = m_motorL1.GetSelectedSensorVelocity();
    double curRightOutput = m_motorR3.GetSelectedSensorVelocity();
    frc::SmartDashboard::PutNumber("VCL_CurLeftOutput", curLeftOutput);
    frc::SmartDashboard::PutNumber("VCL_CurRightOuput", curRightOutput);

    frc::SmartDashboard::PutNumber("VCL_LeftOutputError", leftOutput - curLeftOutput);
    frc::SmartDashboard::PutNumber("VCL_RightOuputError", rightOutput - curRightOutput);

    m_diffDrive.FeedWatchdog();
}

//
//  Gyro
//
void Drivetrain::ResetGyro()
{
    if (m_pigeonValid)
        m_gyro.SetFusedHeading(0.0);
}

degree_t Drivetrain::GetHeadingAngle()
{
    return (m_pigeonValid) ? (m_gyro.GetFusedHeading() * 1_deg) : 0_deg;
}

//
//  Odometry
//
void Drivetrain::ResetOdometry(frc::Pose2d pose)
{
    ResetSensors();
    m_driveSim.SetPose(pose);
    m_odometry.ResetPosition(pose, GetHeadingAngle());
    spdlog::info("Heading angle after odometry reset {}", GetHeadingAngle());
}

///////////////////////////////////////////////////////////////////////////////
//
//  Set Talon brake/coast mode
//
void Drivetrain::SetBrakeMode(bool brakeMode)
{
    m_brakeMode = brakeMode;

    spdlog::info("DT {} Mode", (brakeMode) ? "BRAKE" : "COAST");
    frc::SmartDashboard::PutBoolean("DT_BrakeMode", brakeMode);

    NeutralMode brakeOutput;
    brakeOutput = (brakeMode) ? NeutralMode::Brake : NeutralMode::Coast;
    if (m_talonValidL1)
        m_motorL1.SetNeutralMode(brakeOutput);
    if (m_talonValidL2)
        m_motorL2.SetNeutralMode(brakeOutput);
    if (m_talonValidR3)
        m_motorR3.SetNeutralMode(brakeOutput);
    if (m_talonValidR4)
        m_motorR4.SetNeutralMode(brakeOutput);
}

//
//  Voltage-based tank drive
//
void Drivetrain::TankDriveVolts(volt_t left, volt_t right)
{
    m_diffDrive.Feed();
    if (m_talonValidL1)
        m_motorL1.SetVoltage(left);
    if (m_talonValidR3)
        m_motorR3.SetVoltage(right);
}

void Drivetrain::SyncTalonPIDFromDashboard(void)
{
    m_ramsetePidKf = frc::SmartDashboard::GetNumber("DTR_ramsetePidKf", m_ramsetePidKf);
    m_ramsetePidKp = frc::SmartDashboard::GetNumber("DTR_ramsetePidKp", m_ramsetePidKp);
    m_ramsetePidKi = frc::SmartDashboard::GetNumber("DTR_ramsetePidKi", m_ramsetePidKi);
    m_ramsetePidKd = frc::SmartDashboard::GetNumber("DTR_ramsetePidKd", m_ramsetePidKd);

    if (m_talonValidL1)
    {
        m_motorL1.Config_kF(kSlotIndex, m_ramsetePidKf, 0);
        m_motorL1.Config_kP(kSlotIndex, m_ramsetePidKp, 0);
        m_motorL1.Config_kI(kSlotIndex, m_ramsetePidKi, 0);
        m_motorL1.Config_kD(kSlotIndex, m_ramsetePidKd, 0);
        m_motorL1.SelectProfileSlot(kSlotIndex, kPidIndex);
    }

    if (m_talonValidR3)
    {
        m_motorR3.Config_kF(kSlotIndex, m_ramsetePidKf, 0);
        m_motorR3.Config_kP(kSlotIndex, m_ramsetePidKp, 0);
        m_motorR3.Config_kI(kSlotIndex, m_ramsetePidKi, 0);
        m_motorR3.Config_kD(kSlotIndex, m_ramsetePidKd, 0);
        m_motorR3.SelectProfileSlot(kSlotIndex, kPidIndex);
    }
}

//
bool Drivetrain::MoveIsStopped(void)
{
    bool leftStopped = m_wheelSpeeds.left <= m_tolerance * 1_mps;
    bool rightStopped = m_wheelSpeeds.right <= m_tolerance * 1_mps;

    return (leftStopped && rightStopped);
}
///////////////////////////////////////////////////////////////////////////////
//
//  Trajectory management
//

void Drivetrain::PlotTrajectory(frc::Trajectory trajectory)
{
    // std::vector<frc::Pose2d> poses;
    std::vector<frc::Trajectory::State> states = trajectory.States();
    std::vector<frc::Pose2d> poses;

    for (size_t i = 0; i < states.size(); i++)
        poses.push_back(states[i].pose);

    m_field.GetObject("trajectory")->SetPoses(poses);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Public Interfaces ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//  Reset all sensors - gyro and encoders
//
void Drivetrain::ResetSensors(void)
{
    ResetEncoders();
    ResetGyro();
}

//
//  Set quick turn for curvature drive
//
void Drivetrain::MoveSetQuickTurn(bool quickTurn)
{
    m_isQuickTurn = quickTurn;
}

//
//  Drive stop - used to feed the motors when stopped
//
void Drivetrain::MoveStop()
{
    if (m_talonValidL1 || m_talonValidR3)
        m_diffDrive.TankDrive(0.0, 0.0, false);
}

//
//  Joystick movement during Teleop
//

void Drivetrain::MoveWithJoysticksInit(void)
{
    SetBrakeMode(true);
    m_motorL1.ConfigOpenloopRamp(m_openLoopRampRate, 0);
    m_motorL2.ConfigOpenloopRamp(m_openLoopRampRate, 0);
    m_motorR3.ConfigOpenloopRamp(m_openLoopRampRate, 0);
    m_motorR4.ConfigOpenloopRamp(m_openLoopRampRate, 0);
}

void Drivetrain::MoveWithJoysticks(frc::XboxController *throttleJstick)
{
    double xValue = throttleJstick->GetRightX();
    double yValue = throttleJstick->GetLeftY();
    double xOutput = 0.0;
    double yOutput = 0.0;

    // If joysticks report a very small value, then stick has been centered
    if (fabs(yValue) < 0.05 && fabs(xValue) < 0.05)
        m_throttleZeroed = true;

    // If throttle and steering not centered, use zero outputs until they do
    if (m_throttleZeroed)
    {
        if (m_isQuickTurn)
        {
            xOutput = m_driveQTScaling * (xValue * abs(xValue));
            yOutput = m_driveQTScaling * (yValue * abs(yValue));
        }
        else
        {
            xOutput = m_driveXScaling * (xValue * abs(xValue));
            yOutput = m_driveYScaling * (yValue * abs(yValue));
        }
    }

    if (m_talonValidL1 || m_talonValidR3)
        m_diffDrive.CurvatureDrive(yOutput, xOutput, m_isQuickTurn);
}

void Drivetrain::MoveWithJoysticksEnd(void)
{
    SetBrakeMode(false);
    m_motorL1.ConfigOpenloopRamp(0.0, 0);
    m_motorL2.ConfigOpenloopRamp(0.0, 0);
    m_motorR3.ConfigOpenloopRamp(0.0, 0);
    m_motorR4.ConfigOpenloopRamp(0.0, 0);
}

// Movement during limelight shooting phase
void Drivetrain::MoveWithLimelightInit(bool m_endAtTarget)
{
    // get pid values from dashboard
    m_turnPidKp = frc::SmartDashboard::GetNumber("DTL_TurnPidKp", m_turnPidKp);
    m_turnPidKi = frc::SmartDashboard::GetNumber("DTL_TurnPidKi", m_turnPidKi);
    m_turnPidKd = frc::SmartDashboard::GetNumber("DTL_TurnPidKd", m_turnPidKd);

    m_throttlePidKp = frc::SmartDashboard::GetNumber("DTL_ThrottlePidKp", m_throttlePidKp);
    m_throttlePidKi = frc::SmartDashboard::GetNumber("DTL_ThrottlePidKi", m_throttlePidKi);
    m_throttlePidKd = frc::SmartDashboard::GetNumber("DTL_ThrottlePidKd", m_throttlePidKd);

    m_maxTurn = frc::SmartDashboard::GetNumber("DTL_MaxTurn", m_maxTurn);
    m_maxThrottle = frc::SmartDashboard::GetNumber("DTL_MaxThrottle", m_maxThrottle);
    m_targetAngle = frc::SmartDashboard::GetNumber("DTL_TargetAngle", m_targetAngle);

    m_angleThreshold = frc::SmartDashboard::GetNumber("DTL_AngleThreshold", m_angleThreshold);
    m_distThreshold = frc::SmartDashboard::GetNumber("DTL_DistThreshold", m_distThreshold);
    m_throttleShape = frc::SmartDashboard::GetNumber("DTL_ThrottleShape", m_throttleShape);
    m_vertOffset1 = frc::SmartDashboard::GetNumber("DTL_VertOffset1", m_vertOffset1);
    m_vertOffset2 = frc::SmartDashboard::GetNumber("DTL_VertOffset2", m_vertOffset2);
    m_distance1 = frc::SmartDashboard::GetNumber("DTL_Distance1", m_distance1);
    m_distance2 = frc::SmartDashboard::GetNumber("DTL_Distance2", m_distance2);

    // load in Pid constants to controller
    m_turnPid = frc2::PIDController(m_turnPidKp, m_turnPidKi, m_turnPidKd);
    m_throttlePid = frc2::PIDController(m_throttlePidKp, m_throttlePidKi, m_throttlePidKd);

    // calculate slope and y-intercept
    m_slope = (m_distance2 - m_distance1) / (m_vertOffset2 - m_vertOffset1);
    m_distOffset = m_distance1 - m_slope * m_vertOffset1;
    frc::SmartDashboard::PutNumber("DTL_Slope", m_slope);
    frc::SmartDashboard::PutNumber("DTL_Offset", m_distOffset);

    if (m_endAtTarget)
    {
        m_targetDistance = frc::SmartDashboard::GetNumber("DTL_TargetDistance", m_targetDistance);
    }
    else
    {
        RobotContainer *robotContainer = RobotContainer::GetInstance();
        double ty = robotContainer->m_vision.GetVertOffsetDeg();
        m_limelightDistance = m_slope * ty + m_distOffset;
        m_targetDistance = m_limelightDistance;
    }
}

void Drivetrain::MoveWithLimelightExecute(double tx, double ty, bool tv)
{
    // get turn value - just horizontal offset from target
    double turnOutput = -m_turnPid.Calculate(tx, m_targetAngle);

    // get throttle value
    m_limelightDistance = m_slope * ty + m_distOffset;

    double throttleDistance = m_throttlePid.Calculate(m_limelightDistance, m_targetDistance);
    double throttleOutput = -throttleDistance * pow(cos(turnOutput * wpi::numbers::pi / 180), m_throttleShape);

    // put turn and throttle outputs on the dashboard
    frc::SmartDashboard::PutNumber("DTL_TurnOutput", turnOutput);
    frc::SmartDashboard::PutNumber("DTL_ThrottleOutput", throttleOutput);

    // print out inputs and outputs, intermediate values (slope? throttle distance?)
    spdlog::info(
        "DTL tv {} tx {:.1f} ty {:.1f} | turn {:.2f} throttle {:.2f} | limelightDist {:.1f} throttleDist {:.1f}",
        tv,
        tx,
        ty,
        turnOutput,
        throttleOutput,
        m_limelightDistance,
        throttleDistance);

    // cap max turn and throttle output
    turnOutput = std::clamp(turnOutput, -m_maxTurn, m_maxTurn);
    throttleOutput = std::clamp(throttleOutput, -m_maxThrottle, m_maxThrottle);

    // put turn and throttle outputs on the dashboard
    frc::SmartDashboard::PutNumber("DTL_TurnOutputClamped", turnOutput);
    frc::SmartDashboard::PutNumber("DTL_ThrottleOutputClamped", throttleOutput);

    if (m_talonValidL1 || m_talonValidR3)
        m_diffDrive.ArcadeDrive(throttleOutput, turnOutput, false);
}

bool Drivetrain::MoveWithLimelightIsFinished(double tx, bool tv)
{
    return (
        tv && (fabs(tx) <= m_angleThreshold) && (fabs(m_targetDistance - m_limelightDistance) <= m_distThreshold)
        && MoveIsStopped());
}

void Drivetrain::MoveWithLimelightEnd()
{
    if (m_talonValidL1 || m_talonValidR3)
        m_diffDrive.ArcadeDrive(0.0, 0.0, false);
}

///////////////////////////////////////////////////////////////////////////////
//
//  Autonomous command - Ramsete follower
//
void Drivetrain::RamseteFollowerInit(string pathName, bool resetOdometry)
{
    m_tolerance = frc::SmartDashboard::GetNumber("DT_Tolerance", 0.05);

    m_ramseteB = frc::SmartDashboard::GetNumber("DTR_ramseteB", m_ramseteB);
    m_ramseteZeta = frc::SmartDashboard::GetNumber("DTR_ramseteZeta", m_ramseteZeta);

    // m_leftPid = frc2::PIDController{ m_ramsetePidKp, m_ramsetePidKi, m_ramsetePidKd };
    // m_rightPid = frc2::PIDController{ m_ramsetePidKp, m_ramsetePidKi, m_ramsetePidKd };
    m_ramseteController = frc::RamseteController{ m_ramseteB, m_ramseteZeta };

    // TODO: Not sure if this is really needed or used
    m_leftPid.SetTolerance(m_tolerance);
    m_rightPid.SetTolerance(m_tolerance);

    // Get our trajectory
    // TODO: Move this to be able to load a trajectory while disabled when
    //          the user changes the chooser selection
    std::string outputDirectory = frc::filesystem::GetDeployDirectory();
    outputDirectory.append("/output/" + pathName + ".wpilib.json");
    spdlog::info("DTR Output Directory is: {}", outputDirectory);
    std::ifstream pathFile(outputDirectory.c_str());
    if (pathFile.good())
        spdlog::info("DTR pathFile is good");
    else
        spdlog::error("DTR pathFile not good");

    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(outputDirectory);
    if (!frc::RobotBase::IsReal())
        PlotTrajectory(m_trajectory);
    std::vector<frc::Trajectory::State> trajectoryStates;
    trajectoryStates = m_trajectory.States();
    m_trajTimer.Reset();
    m_trajTimer.Start();

    spdlog::info("DTR Size of state table is {}", trajectoryStates.size());

    for (unsigned int i = 0; i < trajectoryStates.size(); i++)
    {
        frc::Trajectory::State curState = trajectoryStates[i];
        spdlog::info(
            "DTR state time {} Velocity {} Accleration {} Rotation {}",
            curState.t,
            curState.velocity,
            curState.acceleration,
            curState.pose.Rotation().Degrees());
    }

    // This initializes the odometry (where we are)
    SetBrakeMode(false);
    if (resetOdometry)
        ResetOdometry(m_trajectory.InitialPose());
    m_field.SetRobotPose(GetPose());
}

void Drivetrain::RamseteFollowerExecute(void)
{
    // Need to step through the states through the trajectory

    frc::Trajectory::State trajState = m_trajectory.Sample(m_trajTimer.Get());
    frc::Pose2d currentPose = GetPose();

    frc::ChassisSpeeds targetChassisSpeeds = m_ramseteController.Calculate(currentPose, trajState);
    frc::DifferentialDriveWheelSpeeds targetSpeed = m_kinematics.ToWheelSpeeds(targetChassisSpeeds);

    double velLeft = MPSToNativeUnits(targetSpeed.left);
    double velRight = MPSToNativeUnits(targetSpeed.right);

    if (m_talonValidL1)
        m_motorL1.Set(TalonFXControlMode::Velocity, velLeft);
    if (m_talonValidR3)
        m_motorR3.Set(TalonFXControlMode::Velocity, velRight);

    frc::SmartDashboard::PutNumber("DTR_targetLeft", velLeft);
    frc::SmartDashboard::PutNumber("DTR_targetRight", velRight);

    double curVelLeft = m_motorL1.GetSelectedSensorVelocity();
    double curVelRight = m_motorR3.GetSelectedSensorVelocity();

    frc::SmartDashboard::PutNumber("DTR_CurrentLeft", curVelLeft);
    frc::SmartDashboard::PutNumber("DTR_CurrentRight", curVelRight);

    frc::SmartDashboard::PutNumber("DTR_LeftOutputError", velLeft - curVelLeft);
    frc::SmartDashboard::PutNumber("DTR_RightOuputError", velRight - curVelRight);

    // these distLeft and distRight calculations are only accurate for straight paths
    double distLeft = MetersToNativeUnits(trajState.pose.Y());
    double distRight = MetersToNativeUnits(trajState.pose.Y());
    double curDistLeft = m_motorL1.GetSelectedSensorPosition();
    double curDistRight = m_motorR3.GetSelectedSensorPosition();

    frc::SmartDashboard::PutNumber("DTR_targetDistLeft", distLeft);
    frc::SmartDashboard::PutNumber("DTR_targetDistRight", distRight);
    frc::SmartDashboard::PutNumber("DTR_currentDistLeft", curDistLeft);
    frc::SmartDashboard::PutNumber("DTR_currentDistRight", curDistRight);

    frc::SmartDashboard::PutNumber("DTR_DistLeftOutputError", distLeft - curDistLeft);
    frc::SmartDashboard::PutNumber("DTR_DistRightOuputError", distRight - curDistRight);

    m_diffDrive.Feed();

    spdlog::info(
        "DTR cur XYR {:.2f} {:.2f} {:.1f} | targ XYR {:.2f} {:.2f} {:.1f} | chas XYO {:.2f} {:.2f} {:.1f} | targ vel LR {:.2f} {:.2f} | cur vel LR {:.2f} {:.2f}",
        currentPose.X().to<double>(),
        currentPose.Y().to<double>(),
        currentPose.Rotation().Degrees().to<double>(),
        trajState.pose.X().to<double>(),
        trajState.pose.Y().to<double>(),
        trajState.pose.Rotation().Degrees().to<double>(),
        targetChassisSpeeds.vx.to<double>(),
        targetChassisSpeeds.vy.to<double>(),
        targetChassisSpeeds.omega.to<double>(),
        velLeft,
        velRight,
        curVelLeft,
        curVelRight);
}

bool Drivetrain::RamseteFollowerIsFinished(void)
{
    return (
        (m_trajTimer.Get() >= m_trajectory.TotalTime())
        && (abs(m_motorL1.GetSelectedSensorVelocity()) <= 0 + m_tolerance)
        && (abs(m_motorL1.GetSelectedSensorVelocity()) <= 0 + m_tolerance));
}

void Drivetrain::RamseteFollowerEnd(void)
{
    m_trajTimer.Stop();
    SetBrakeMode(true);
    TankDriveVolts(0.0_V, 0.0_V);

    double curVelLeft = m_motorL1.GetSelectedSensorVelocity();
    double curVelRight = m_motorR3.GetSelectedSensorVelocity();

    frc::SmartDashboard::PutNumber("DTR_CurrentLeft", curVelLeft);
    frc::SmartDashboard::PutNumber("DTR_CurrentRight", curVelRight);

    // added for PID tuning, may be removed once completed
    frc::SmartDashboard::PutNumber("DTR_LeftOutputError", 0 - curVelLeft);
    frc::SmartDashboard::PutNumber("DTR_RightOuputError", 0 - curVelRight);
}

void Drivetrain::DriveBackward(double tx, double ty, bool tv) {}

frc::Pose2d Drivetrain::GetPose()
{
    return m_odometry.GetPose();
}