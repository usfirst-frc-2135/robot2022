
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.Constants.INConsts.INMode;
import frc.robot.Constants.TCConsts.TCMode;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TowerConveyor;

/**
 *
 */
public class IntakingStop extends SequentialCommandGroup
{
  public IntakingStop(Intake intake, FloorConveyor fConv, TowerConveyor tConv)
  {
    setName("IntakingStop");
    addCommands(
        // Add Commands here:
        // Also add parallel commands using the
        //
        new IntakeRun(intake, INMode.INTAKE_STOP), new FloorConveyorRun(FCMode.FCONVEYOR_STOP, fConv),
        new TowerConveyorRun(tConv, TCMode.TCONVEYOR_STOP)

    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
