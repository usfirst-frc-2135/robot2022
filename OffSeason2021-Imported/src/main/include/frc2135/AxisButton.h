// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/GenericHID.h>
#include <frc2/command/button/Button.h>

namespace frc2135
{
    /**
 * A class used to bind command scheduling to joystick axis presses.  Can be
 * composed with other buttons with the operators in Trigger.
 *
 * @see Trigger
 */
    class AxisButton : public frc2::Button
    {
    public:
        /**
   * Creates a AxisButton that commands can be bound to.
   *
   * @param joystick The joystick on which the button is located.
   * @param buttonNumber The number of the button on the joystic.
   */
        explicit AxisButton(frc::GenericHID *joystick, int buttonNumber) :
            Button([joystick, buttonNumber] { return (joystick->GetRawAxis(buttonNumber) > 0.5); })
        {
        }
    };
} // namespace frc2135
