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

#include <frc/filter/MedianFilter.h>
#include <networktables/NetworkTable.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include <frc2/command/SubsystemBase.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

/**
 *
 *
 * @author ExampleAuthor
 */
class Vision : public frc2::SubsystemBase
{
private:
    // It's desirable that everything possible is private except
    // for methods that implement subsystem capabilities
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    std::shared_ptr<nt::NetworkTable> table;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // Declare module variables
    double m_targetHorizAngle; // Horizontal Offset from Crosshair to Target (-27 degrees to 27 degrees)
    double m_targetVertAngle;  // Vertical Offset from Crosshair to Target (-20.5 degrees to 20.5 degrees)
    double m_targetArea;       // Target Area (0% of image to 100% of image)
    double m_targetSkew;       // Target Skew or rotation (-90 degrees to 0 degrees)
    bool m_targetValid;        // Target Valid or not

    //variables in inches to calculate limelight distance
    double m_distance1;
    double m_distance2;
    double m_slope;
    double m_vertOffset1;
    double m_vertOffset2;
    double m_distOffset;
    double m_distLight;
    double m_setPointDistance;

public:
    Vision();
    void Periodic() override;
    void SimulationPeriodic() override;
    void Initialize(void);
    double CalculateDist(void);
    void SyncStateFromDashboard(void);
    frc::MedianFilter<double> m_yfilter;

    // Limelight LED mode states
    typedef enum ledMode_e
    {
        LED_CUR_MODE = 0,
        LED_OFF = 1,
        LED_BLINK = 2,
        LED_ON = 3,
    } ledMode_e;

    // Camera Limelight streaming
    typedef enum camStream_e
    {
        STANDARD = 0,
        PIP_MAIN = 1,
        PIP_SECONDARY = 2,
    } camStream_e;

    double GetHorizOffsetDeg();
    double GetVertOffsetDeg();
    double GetTargetArea();
    double GetTargetSkew();
    bool GetTargetValid();
    void SetLEDMode(ledMode_e mode);

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
};
