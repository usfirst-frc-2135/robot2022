
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CLConsts.Height;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class Climber5RotateIntoL3 extends SequentialCommandGroup
{
  public Climber5RotateIntoL3(Climber climber)
  {
    setName("ClimberRotateIntoL3");

    addCommands(new ParallelDeadlineGroup(new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
        new ClimberMoveToHeight(climber, Height.NOCHANGE_HEIGHT), new ClimberSetGatehook(climber, false))
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
