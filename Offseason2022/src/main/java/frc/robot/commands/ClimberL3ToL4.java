
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;

/**
 *
 */
public class ClimberL3ToL4 extends SequentialCommandGroup
{
  public ClimberL3ToL4(Climber climber, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter)
  {
    addCommands(
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
