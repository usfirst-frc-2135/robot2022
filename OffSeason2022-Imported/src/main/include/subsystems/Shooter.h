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
#include <frc/Encoder.h>
#include <frc/filter/LinearFilter.h>
#include <frc/Solenoid.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/moment_of_inertia.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "ctre/Phoenix.h"

#include <frc2/command/SubsystemBase.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

using namespace units::length;
using namespace units::moment_of_inertia;
using namespace units::time;

/**
 *
 *
 * @author ExampleAuthor
 */
class Shooter : public frc2::SubsystemBase
{
private:
    // It's desirable that everything possible is private except
    // for methods that implement subsystem capabilities

#ifdef __FRC_ROBORIO__
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    WPI_TalonFX m_motorSH11{ 11 };
    WPI_TalonFX m_motorSH10{ 10 };

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
#else
    WPI_TalonSRX m_motorSH10{ 10 };
    WPI_TalonSRX m_motorSH11{ 11 };
#endif

    frc::Solenoid m_flashlight{ 0, frc::PneumaticsModuleType::CTREPCM, 7 };

    frc::LinearFilter<double> m_feederFilter = frc::LinearFilter<double>::SinglePoleIIR(0.1, 0.02_s);
    frc::LinearFilter<double> m_flywheelFilter = frc::LinearFilter<double>::SinglePoleIIR(0.1, 0.02_s);

    // Sensors

    // Simulated quadrature encoders, since TalonFX is not supported
    frc::Encoder m_feederEncoder{ 5, 6 };
    frc::Encoder m_flywheelEncoder{ 7, 8 };
    frc::sim::EncoderSim m_feederEncoderSim{ m_feederEncoder };
    frc::sim::EncoderSim m_flywheelEncoderSim{ m_flywheelEncoder };
    frc::sim::FlywheelSim m_feederSim{
        frc::LinearSystemId::FlywheelSystem(frc::DCMotor::Falcon500(1), 1.0 * 1.0_kg_sq_m, 1.0),
        frc::DCMotor::Falcon500(1),
        1.5
    };
    frc::sim::FlywheelSim m_flywheelSim{
        frc::LinearSystemId::FlywheelSystem(frc::DCMotor::Falcon500(1), 1.0 * 1.0_kg_sq_m, 1.0),
        frc::DCMotor::Falcon500(1),
        1.5
    };

    const int m_shooterDebug = 1; // DEBUG flag to disable/enable extra logging calls
    const int kPidIndex = 0;      // PID index for primary sensor
    const int kSlotIndex = 0;     // PID slot index for sensors
    const int kCANTimeout = 10;   // CAN timeout in msec to wait for response

    const double kFalconEncoderCPR = 2048; // CPR is 2048 from Falcon 500 Manual

    const double kFeederGearRatio = (1.5 / 1); // Reduction of 1.5/1
    const double kFeederCPR = kFalconEncoderCPR * kFeederGearRatio;

    const double kFlywheelGearRatio = (1.0 / 1.0); // No reduction 1:1
    const double kFlywheelCPR = kFalconEncoderCPR * kFlywheelGearRatio;

    // Declare module variables
    bool m_talonValidSH10; // Health indicator for shooter talon 10
    bool m_talonValidSH11; // Health indicator for shooter talon 11

    // Configuration file parameters

    double m_feederPidKf;           // Feeder PID force constant
    double m_feederPidKp;           // Feeder PID proportional constant
    double m_feederPidKi;           // Feeder PID integral constant
    double m_feederPidKd;           // Feeder PID derivative constant
    double m_feederNeutralDeadband; // Feeder PID neutral deadband in percent
    double m_feederTargetRPM;       // Target feeder RPM for shooting

    double m_flywheelPidKf;           // Flywheel PID force constant
    double m_flywheelPidKp;           // Flywheel PID proportional constant
    double m_flywheelPidKi;           // Flywheel PID integral constant
    double m_flywheelPidKd;           // Flywheel PID derivative constant
    double m_flywheelNeutralDeadband; // Flywheel PID neutral deadband in percent
    double m_flywheelTargetRPM;       // Target flywheel RPM for shooting

    // Measured RPM
    double m_feederCurrentRPM;   // Current feeder RPM
    double m_flywheelCurrentRPM; // Current flywheel RPM

    double m_toleranceRPM; // Allowed variation from target RPM
    int m_state;           // Saved shooter state

    // Conversion functions between RPM and Output and CTRE Native Units / 100ms
    double FeederRPMToNative(double rpm);
    double NativeToFeederRPM(double native);
    double FlywheelRPMToNative(double rpm);
    double NativeToFlywheelRPM(double native);

public:
    Shooter();

    void Periodic() override;
    void SimulationPeriodic() override;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    typedef enum shooterSpeed_e
    {
        SHOOTERSPEED_STOP = 0,    // Stop shooter
        SHOOTERSPEED_FORWARD = 1, // Shooter velocity
    } shooterSpeed_e;

    void Initialize(void);
    void FaultDump(void);

    void SetShooterSpeed(int state);
    void FlashlightOn(bool onState);
    bool AtDesiredRPM();
};
