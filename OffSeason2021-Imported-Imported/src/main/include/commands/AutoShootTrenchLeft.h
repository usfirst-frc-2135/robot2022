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
#include "subsystems/Drivetrain.h"
#include "subsystems/FloorConveyor.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/VerticalConveyor.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include <frc2/command/SequentialCommandGroup.h>

/**
 *
 *
 * @author ExampleAuthor
 */
class AutoShootTrenchLeft : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutoShootTrenchLeft>
{
public:
    explicit AutoShootTrenchLeft(
        Drivetrain *m_drivetrain,
        Intake *m_intake,
        FloorConveyor *m_floorConveyor,
        VerticalConveyor *m_verticalConveyor,
        Shooter *m_shooter);

    bool RunsWhenDisabled() const override;

private:
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES
};
