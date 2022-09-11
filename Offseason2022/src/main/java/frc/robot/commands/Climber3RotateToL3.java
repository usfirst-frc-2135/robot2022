
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

    addCommands(
        // Add Commands here:

        // @formatter:off
      new ParallelCommandGroup(
          new ParallelDeadlineGroup(
              new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
              new ClimberMoveToHeight(climber, Height.ROTATE_L3_HEIGHT)), 
          new ClimberSetGatehook(climber, true)
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
