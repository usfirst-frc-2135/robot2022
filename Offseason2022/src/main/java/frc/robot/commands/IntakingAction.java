
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.Constants.INConsts.INMode;
import frc.robot.Constants.TCConsts.TCMode;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TowerConveyor;

/**
 *
 */
public class IntakingAction extends SequentialCommandGroup
{
  public IntakingAction(Intake intake, FloorConveyor fConv, TowerConveyor tConv)
  {
    setName("IntakingAction");
    addCommands(
        // Add Commands here:
        // Also add parallel commands using the
        //
        new IntakeDeploy(true), // Deploy intake
        new IntakeRun(intake, INMode.INTAKE_ACQUIRE), //
        new FloorConveyorRun(FCMode.FCONVEYOR_EXPEL, fConv),
        new SequentialCommandGroup(new TowerConveyorRun(tConv, TCMode.TCONVEYOR_ACQUIRE_SLOW), //
            new WaitUntilCommand(tConv::isCargoDetected),
            //
            new TowerConveyorRun(tConv, TCMode.TCONVEYOR_STOP))

    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
