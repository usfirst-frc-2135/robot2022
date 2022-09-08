
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class DriveLimelightStop extends SequentialCommandGroup
{
  private final Drivetrain m_drivetrain;

  public DriveLimelightStop(Drivetrain drivetrain)
  {
    m_drivetrain = drivetrain;
    setName("DriveLimelightStop");

    DataLogManager.log(getSubsystem( ) + ": DriveLimelightStop");

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
