
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CLConsts.Height;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class Climber2ClimbToL2 extends SequentialCommandGroup
{
  public Climber2ClimbToL2(Climber climber)
  {
    addCommands(
        // Add Commands here:

        // @formatter:off
      new ParallelDeadlineGroup(
          new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
          new ClimberMoveToHeight(climber, Height.STOW_HEIGHT) 
      ),
      new ClimberSetGatehook(climber, false),
      new WaitCommand(1.5), // see if necessary
      new ParallelDeadlineGroup(
          new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
          new ClimberMoveToHeight(climber, Height.GATEHOOK_REST_HEIGHT) 
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
