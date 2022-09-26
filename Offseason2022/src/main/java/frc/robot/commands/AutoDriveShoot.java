
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
public class AutoDriveShoot extends SequentialCommandGroup
{
  public AutoDriveShoot(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    setName("AutoDriveShoot");

    String m_pathname1 = AUTOConstants.kDriveShoot_path1;
    String m_pathname2 = AUTOConstants.kDriveShoot_path2;

    DataLogManager.log("AutoDriveShoot pathname 1 : " + m_pathname1);
    DataLogManager.log("AutoDriveShoot pathname 2 : " + m_pathname2);

    addCommands(
        // Add Commands here:

        //@formatter:off
        new PrintCommand("AUTO: Use programmable delay from dashboard before starting"),
        new ParallelDeadlineGroup(
          new AutoWait(AutoTimer.TIMER1), 
          new AutoStop(drivetrain)
        ),

        new PrintCommand("AUTO: Deploy the intake"),
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, true), 
          new AutoStop(drivetrain)
        ),

        new PrintCommand("AUTO: Run the first path to shooting position, prime shooter"),
        new ParallelCommandGroup(
          new ParallelDeadlineGroup(
            new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
            new AutoDrivePath(drivetrain, m_pathname1, true)
            ), 
          new ScoringPrime(shooter, vision)
        ),

        new PrintCommand("AUTO: Shoot into upper hub for specified time"),
        new ParallelDeadlineGroup(
          new ScoringActionUpperHub(intake, fConv, tConv, shooter, 1.5), 
          new AutoStop(drivetrain)
        ),

        new PrintCommand("AUTO: Retract the intake"),
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, false), 
          new AutoStop(drivetrain)
        ),

        new PrintCommand("AUTO: Delay before next move"),
        new ParallelDeadlineGroup(
          new WaitCommand(1),
          new AutoStop(drivetrain)
        ),

        new PrintCommand("AUTO: Run the second path off the tarmac"),
        new ScoringStop(intake, fConv, tConv, shooter, vision), 
        new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
          new AutoDrivePath(drivetrain, m_pathname2, false)
        ),

        new PrintCommand("AUTO: Sit still while feeding motors"),
        new AutoStop(drivetrain)
        //@formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
