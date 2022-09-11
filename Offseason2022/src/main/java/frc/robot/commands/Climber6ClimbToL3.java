
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
public class Climber6ClimbToL3 extends SequentialCommandGroup
{
  public Climber6ClimbToL3(Climber climber)
  {
    setName("ClimberClimbToL3");

    addCommands(
        // Add Commands here:

        // @formatter:off
      new ClimberSetGatehook(climber, false),
      new ParallelDeadlineGroup(
          new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
          new ClimberMoveToHeight(climber, Height.STOW_HEIGHT)),
      new WaitCommand(1.5), 
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
