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

#include "ctre/Phoenix.h"

#include <frc/smartdashboard/SendableChooser.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include <frc2/command/SubsystemBase.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

/**
 *
 *
 * @author ExampleAuthor
 */
class LED : public frc2::SubsystemBase
{
private:
    // It's desirable that everything possible is private except
    // for methods that implement subsystem capabilities
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    CANdle m_candle{ 0 };

    // Declare constants

    // Declare module variables
    int m_previousColor = 0;

    // Store what the last hue of the first pixel is
    // int firstPixelHue = 0;
    //bool m_sourceEnabled = true;

    void SendRGBToLED(int rgbRed, int rgbGreen, int rgbBlue);

public:
    LED();

    void Periodic() override;
    void SimulationPeriodic() override;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    typedef enum LEDColor_e
    {
        LEDCOLOR_OFF = -1,
        LEDCOLOR_WHITE = 0,
        LEDCOLOR_RED = 1,
        LEDCOLOR_ORANGE = 2,
        LEDCOLOR_YELLOW = 3,
        LEDCOLOR_GREEN = 4,
        LEDCOLOR_BLUE = 5,
        LEDCOLOR_PURPLE = 6,
        LEDCOLOR_DASH = 7
    } LEDColor_e;

    

    void Initialize(void);
    void SetColor(int color);
    void SetShooterColor(int color);
    void SetLLColor(int color);

    frc::SendableChooser<LEDColor_e> m_ledChooser;
};
