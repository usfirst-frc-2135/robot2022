
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.Constants.INConsts.INMode;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TowerConveyor;

/**
 *
 */
public class ExhaustingAction extends SequentialCommandGroup
{
  public ExhaustingAction(Intake intake, FloorConveyor fConv, TowerConveyor tConv)
  {
    addCommands(
        // Add Commands here:

        // @formatter:off
        new IntakeDeploy(intake, true), 
        new IntakeRun(intake, INMode.INTAKE_EXPEL),
        new FloorConveyorRun(fConv, FCMode.FCONVEYOR_EXPEL_FAST)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
