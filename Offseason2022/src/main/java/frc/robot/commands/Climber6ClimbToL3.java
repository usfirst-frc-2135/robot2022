
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

    addCommands(new ClimberSetGatehook(climber, false),
        new ParallelDeadlineGroup(new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
            new ClimberMoveToHeight(climber, Height.STOW_HEIGHT)),
        new WaitCommand(1.5), new ParallelDeadlineGroup(new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
            new ClimberMoveToHeight(climber, Height.GATEHOOK_REST_HEIGHT))

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
