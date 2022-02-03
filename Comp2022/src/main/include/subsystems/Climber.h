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
#include <frc/XboxController.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "Constants.h"
#include "ctre/Phoenix.h"

#include <frc/Solenoid.h>
#include <frc2/command/SubsystemBase.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

using namespace units::time;
using namespace ClimberConstants;
/**
 *
 *
 * @author ExampleAuthor
 */
class Climber : public frc2::SubsystemBase
{
private:
    // It's desirable that everything possible is private except
    // for methods that implement subsystem capabilities

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    frc::Solenoid m_gatehook{ 0, frc::PneumaticsModuleType::CTREPCM, 1 };
    WPI_TalonFX m_motorCL14{ 14 };
    WPI_TalonFX m_motorCL15{ 15 };

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    // Declare constants
    const int m_climberDebug = 0; // DEBUG flag to disable/enable extra logging calls
    const int kCANTimeout = 10;   // CAN timeout in msec to wait for response

    // Declare module variables
    bool m_talonValidCL14; // Health indicator for climber Talon 14
    bool m_talonValidCL15; // Health indicator for climber Talon 15
    double m_deadband = 0.2;

    double m_targetInches;   // Target inches of height that are requested of the climber
    double m_curInches;      // Current elevator height in inches
    bool m_calibrated;       // Indicates whether the climber has been calibrated
    bool m_isMoving = false; // State of whether the climber is moving or stationary

    frc::Timer m_safetyTimer; // Safety timer for use in elevator
    second_t m_safetyTimeout; // Seconds that the timer ran before stopping

    // Config file parameters
    double m_peakOut;         // Climber maximum speed during movement
    int m_velocity;           // Climber motion velocity
    int m_acceleration;       // Climber motion acceleration
    int m_sCurveStrength;     // Climber motion S curve smoothing strength
    double m_pidKf;           // Climber PID force constant
    double m_pidKp;           // Climber PID proportional constant
    double m_pidKi;           // Climber PID integral constant
    double m_pidKd;           // Climber PID derivative constant
    double m_neutralDeadband; // Climber PID neutral deadband in percent
    double m_CLRampRate;      // Climber PID ramp rate
    int m_CLAllowedError;     // Climber PID allowable closed loop error in counts
    double m_toleranceInches; // Climber PID tolerance in inches

    double m_climberMaxHeight; // Climber maximum allowable height
    double m_climberMinHeight; // Climber minimum allowable height
    double m_stateHeight;
    double m_stowHeight; //
    double m_extendL2;   //
    double m_rotateL3;   //
    double m_extendL3;

public:
    Climber();

    void Periodic() override;
    void SimulationPeriodic() override;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    enum
    {                         // Climber subsystem movement states
        NOCHANGE_HEIGHT = -1, // No change in climber height--maintain current position
        STOW_HEIGHT = 0,      // Move to stow height
        EXTEND_L2_HEIGHT = 1, // Move to extend to L2 height
        ROTATE_L3_HEIGHT = 2, // Move to rotate to L3 height
        EXTEND_L3_HEIGHT = 3, // Move to extend to L3 height
    };

    void Initialize(void);
    void FaultDump(void);
    void MoveClimberWithJoysticks(frc::XboxController *operatorController);
    void SetGateHook(bool hookClosed);
    void SetClimberStopped(void);

    enum
    {
        CLIMBER_INIT = -2,
        CLIMBER_DOWN = -1,
        CLIMBER_STOPPED = 0,
        CLIMBER_UP = 1
    };

    double InchesToCounts(double inches);
    double CountsToInches(int counts);
    void Calibrate();

    // Motion Magic - Moving to a position
    void MoveClimberDistanceInit(int state);
    bool MoveClimberDistanceIsFinished(void);
};
