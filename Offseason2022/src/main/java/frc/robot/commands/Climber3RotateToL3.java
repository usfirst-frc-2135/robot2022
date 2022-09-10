
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CLConsts.Height;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class Climber3RotateToL3 extends SequentialCommandGroup
{
  public Climber3RotateToL3(Climber climber)
  {
    setName("ClimberRotateToL3");

    addCommands(new ParallelCommandGroup(new ParallelDeadlineGroup(new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
        new ClimberMoveToHeight(climber, Height.ROTATE_L3_HEIGHT)), new ClimberSetGatehook(climber, true))
    // Add Commands here:
    // Also add parallel commands using the
    //
    // addCommands(
    // new command1(argsN, subsystem),
    // parallel(
    // new command2(argsN, subsystem),
    // new command3(argsN, subsystem)
    // )
    // );

    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
