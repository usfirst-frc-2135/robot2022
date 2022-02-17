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

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "ctre/Phoenix.h"

#include <frc/Solenoid.h>
#include <frc2/command/SubsystemBase.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

/**
 *
 *
 * @author ExampleAuthor
 */
class Intake : public frc2::SubsystemBase
{
private:
    // It's desirable that everything possible is private except
    // for methods that implement subsystem capabilities
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    frc::Solenoid m_position{ 0, frc::PneumaticsModuleType::CTREPCM, 0 };
    
#if 0 // TODO: set to 1 for Grogu, set to 0 for 2022 bots
    WPI_TalonSRX m_motorIN6{ 6 };
#else
    WPI_TalonFX m_motorIN6{ 6 };
#endif

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // Declare constants
    const int m_intakeDebug = 0; // DEBUG flag to disable/enable extra logging calls
    const int kCANTimeout = 10;  // CAN timeout in msec to wait for response

    // Declare module variables
    bool m_talonValidIN6; // Health indicator for intake Talon 6

    double m_acquireSpeed;
    double m_expelSpeed;

public:
    Intake();

    void Periodic() override;
    void SimulationPeriodic() override;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // Intake motor control states
    typedef enum intakeDirection_e
    {
        INTAKE_STOP = 0,
        INTAKE_ACQUIRE = 1,
        INTAKE_EXPEL = -1,
    } intakeDirection_e;

    void Initialize(void);
    void FaultDump(void);

    void SetIntakeSpeed(int mode);
    void SetDeployerSolenoid(bool extended);
};
