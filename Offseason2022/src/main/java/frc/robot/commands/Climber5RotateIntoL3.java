
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
    setName("Climber5RotateIntoL3");

    addCommands(
        // Add Commands here:

        // @formatter:off
      new ParallelDeadlineGroup(
          new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
          new ClimberMoveToHeight(climber, Height.NOCHANGE_HEIGHT), 
          new ClimberSetGatehook(climber, false)
      )
      // @formatter:on

    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
