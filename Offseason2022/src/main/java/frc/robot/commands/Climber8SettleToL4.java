
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CLConsts.CLHeight;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class Climber8SettleToL4 extends SequentialCommandGroup
{
  public Climber8SettleToL4(Climber climber)
  {
    setName("Climber8SettleToL4");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new ParallelDeadlineGroup(
            new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
            new ClimberMoveToHeight(climber, CLHeight.HEIGHT_STOW), 
            new ClimberSetGatehook(climber, false)
        ),
        new WaitCommand(0.5), 
        new ParallelDeadlineGroup(
            new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
            new ClimberMoveToHeight(climber, CLHeight.HEIGHT_GATEHOOK_REST)
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
