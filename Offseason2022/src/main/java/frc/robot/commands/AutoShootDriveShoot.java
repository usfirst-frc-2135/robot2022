
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AUTOConstants;
import frc.robot.Constants.AUTOConstants.AutoTimer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class AutoShootDriveShoot extends SequentialCommandGroup
{
  private String m_pathname1 = AUTOConstants.kShootDriveShoot_path1;
  private String m_pathname2 = AUTOConstants.kShootDriveShoot_path2;
  private String m_pathname3 = AUTOConstants.kShootDriveShoot_path3;
  private String m_pathname4 = AUTOConstants.kShootDriveShoot_path4;

  public AutoShootDriveShoot(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {

    setName("AutoShootDriveShoot");

    DataLogManager.log("AutoShootDriveShoot pathname 1 : " + m_pathname1);
    DataLogManager.log("AutoShootDriveShoot pathname 2 : " + m_pathname2);
    DataLogManager.log("AutoShootDriveShoot pathname 3 : " + m_pathname3);
    DataLogManager.log("AutoShootDriveShoot pathname 4 : " + m_pathname4);

    addCommands(
        // Add Commands here:
        // @formatter:off
      
      new AutoWait(AutoTimer.TIMER1),
      new ParallelDeadlineGroup( new IntakeDeploy(intake, true), new AutoStop(drivetrain) ),
      new ParallelCommandGroup(
          new ParallelDeadlineGroup(
              new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
              new AutoDrivePath(drivetrain, m_pathname1, true) ),
          new ScoringPrime(shooter, vision) ),
      new ParallelDeadlineGroup( new ScoringActionUpperHub(intake, fConv, tConv, shooter, 1), new AutoStop(drivetrain) ),
      new ParallelDeadlineGroup(
          new ParallelDeadlineGroup(
              new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
              new AutoDrivePath(drivetrain, m_pathname2, false) ),
          new IntakingAction(intake, fConv, tConv),
          new ScoringPrime(shooter, vision) ),
      new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
          new AutoDrivePath(drivetrain, m_pathname3, false) ),
      new ParallelDeadlineGroup( new ScoringActionUpperHub(intake, fConv, tConv, shooter, 2), new AutoStop(drivetrain) ),
      new ParallelDeadlineGroup( new IntakeDeploy(intake, false), new AutoStop(drivetrain) ),
      new WaitCommand(1),
      new ParallelCommandGroup(
          new ParallelDeadlineGroup(
              new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
              new AutoDrivePath(drivetrain, m_pathname4, false) ),
          new ScoringStop(intake, fConv, tConv, shooter, vision) )      
          
      // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
