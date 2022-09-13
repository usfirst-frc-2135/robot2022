
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class DriveLimelightShoot extends SequentialCommandGroup
{
  private final Drivetrain m_drivetrain;

  public DriveLimelightShoot(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    m_drivetrain = drivetrain;
    setName("DriveLimelightShoot");

    DataLogManager.log(getSubsystem( ) + ": DriveLimelightShoot");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new ParallelCommandGroup(
            new DriveLimelight(drivetrain, vision, false),
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(drivetrain::driveWithLimelightIsFinished),
                    new ScoringPrime(shooter, vision)), 
                new ScoringActionHighHub(120, shooter)))
        
        // @formatter:on

    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
