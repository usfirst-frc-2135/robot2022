
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.AUTOConstants;

/**
 *
 */
public class AutoDriveShoot extends SequentialCommandGroup
{
  public AutoDriveShoot(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    setName("AutoDriveShoot");

    String m_pathname1 = AUTOConstants.kDriveShoot_path1;
    String m_pathname2 = AUTOConstants.kDriveShoot_path1;

    DataLogManager.log("AutoDriveShoot pathname 1 : " + m_pathname1);
    DataLogManager.log("AutoDriveShoot pathname 2 : " + m_pathname2);

    addCommands(// Sequential command
        // Wait timer set in SmartDasboard
        // Add Commands here:

        //@formatter:off
        new AutoWait(1), 
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, true), 
          new AutoStop(drivetrain)
        ),
        new ParallelCommandGroup(
          new ParallelDeadlineGroup(
            new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
            new AutoDrivePath(drivetrain, m_pathname1, true)
            ), 
          new ScoringPrime(shooter, vision)
        ), 
        new ParallelDeadlineGroup(
          new ScoringActionUpperHub(intake, fConv, tConv, shooter, 1.5), 
          new AutoStop(drivetrain)
        ),
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, false), 
          new AutoStop(drivetrain)
        ), 
        new WaitCommand(1),
        new ScoringStop(intake, fConv, tConv, shooter, vision), 
        new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
          new AutoDrivePath(drivetrain, m_pathname2, false))
        //@formatter:on

    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
