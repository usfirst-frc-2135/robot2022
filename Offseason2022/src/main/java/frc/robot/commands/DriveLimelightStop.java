
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class DriveLimelightStop extends SequentialCommandGroup
{
  private final Drivetrain m_drivetrain;

  public DriveLimelightStop(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    m_drivetrain = drivetrain;
    setName("DriveLimelightStop");

    DataLogManager.log(getSubsystem( ) + ": DriveLimelightStop");

    addCommands(new ParallelDeadlineGroup(new ScoringStop(intake, fConv, tConv, shooter, vision), new AutoStop(drivetrain))
    // Add Commands here:
    // Also add parallel commands using the
    //
    // addCommands(
    // new command1(argsN, subsystem),
    // parallel(
    // new command2(argsN, subsystem),
    // new command3(argsN, subsystem)
    // )
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
