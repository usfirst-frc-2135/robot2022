
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
public class Climber7ClimbToL4 extends SequentialCommandGroup
{
  public Climber7ClimbToL4(Climber climber)
  {
    setName("ClimberClimbToL4");

    addCommands(new ParallelDeadlineGroup(new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
        new ClimberMoveToHeight(climber, Height.RAISE_L4_HEIGHT), new ClimberSetGatehook(climber, false))
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
