// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include <frc/AddressableLED.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "subsystems/LED.h"

#include <frc/smartdashboard/SmartDashboard.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

LED::LED()
{
    SetName("LED");
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SetSubsystem("LED");

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // m_led.SetLength(kLEDStringLength);
    // m_led.SetData(m_ledBuffer);
    // m_led.Start();

    SetColor(LEDCOLOR_BLUE);

    // Add options for colors in SmartDashboard
    m_ledChooser.AddOption("LED_Off", LEDCOLOR_OFF);
    m_ledChooser.AddOption("LED_White", LEDCOLOR_WHITE);
    m_ledChooser.AddOption("LED_Red", LEDCOLOR_RED);
    m_ledChooser.AddOption("LED_Orange", LEDCOLOR_ORANGE);
    m_ledChooser.AddOption("LED_Yellow", LEDCOLOR_YELLOW);
    m_ledChooser.AddOption("LED_Green", LEDCOLOR_GREEN);
    m_ledChooser.AddOption("LED_Blue", LEDCOLOR_BLUE);
    m_ledChooser.AddOption("LED_Purple", LEDCOLOR_PURPLE);

    frc::SmartDashboard::PutData("LED_Color", &m_ledChooser);
}

void LED::Periodic()
{
    // Put code here to be run every loop

    // if (m_sourceEnabled)
    //     SetColor(m_ledChooser.GetSelected());
    // else
    //     SetColor(LEDCOLOR_OFF);
}

void LED::SimulationPeriodic()
{
    // This method will be called once per scheduler run when in simulation
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// Put methods for controlling this subsystem
// here. Call these from Commands.

void LED::Initialize()
{
    spdlog::info("LED Init");

    //Default to a length of 60, start empty output
    //Length is expensive to set, so only set it once, then just update data
}

// Send RGB values to change color of each LED in string
void LED::SendRGBToString(int rgbRed, int rgbGreen, int rgbBlue)
{
    for (int i = 0; i < kLEDStringLength; i++)
    {
        m_ledBuffer[i].SetRGB(rgbRed, rgbGreen, rgbBlue);
    }

    // m_led.SetData(m_ledBuffer);
}

// Set color of LED string
void LED::SetColor(int color)
{
    const char *strName;

    if (color == (LEDCOLOR_DASH))
    {
        color = m_ledChooser.GetSelected();
    }

    switch (color)
    {
        default:
        case LEDCOLOR_OFF:
            strName = "OFF";
            SendRGBToString(0, 0, 0); //black
            break;
        case LEDCOLOR_WHITE:
            strName = "WHITE";
            SendRGBToString(255, 255, 255); //white
            break;
        case LEDCOLOR_RED:
            strName = "RED";
            SendRGBToString(255, 0, 0); //red
            break;
        case LEDCOLOR_ORANGE:
            strName = "ORANGE";
            SendRGBToString(255, 80, 0); //orange
            break;
        case LEDCOLOR_YELLOW:
            strName = "YELLOW";
            SendRGBToString(255, 255, 0); //yellow
            break;
        case LEDCOLOR_GREEN:
            strName = "GREEN";
            SendRGBToString(0, 255, 0); //green
            break;
        case LEDCOLOR_BLUE:
            strName = "BLUE";
            SendRGBToString(0, 0, 255); //blue
            break;
        case LEDCOLOR_PURPLE:
            strName = "PURPLE";
            SendRGBToString(255, 0, 255); //purple
            break;
    }

    // if (m_previousColor != color)
    {
        spdlog::info("LED Color Set to {}", strName);
        m_previousColor = color;
    }
}
