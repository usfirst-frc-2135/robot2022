
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CLConsts.CLHeight;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class Climber7ClimbToL4 extends SequentialCommandGroup
{
  public Climber7ClimbToL4(Climber climber)
  {
    setName("Climber7ClimbToL4");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new ParallelDeadlineGroup(
            new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
            new ClimberMoveToHeight(climber, CLHeight.HEIGHT_RAISE_L4)
        ), 
        new ClimberSetGatehook(climber, false)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
