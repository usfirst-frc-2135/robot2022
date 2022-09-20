
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

/**
 *
 */
public class AutoDrive extends SequentialCommandGroup
{
  public AutoDrive(Drivetrain drivetrain, Intake intake)
  {
    setName("AutoDrive");

    String m_pathname = "startToOffTarmac";

    DataLogManager.log("AutoDriveShoot pathname 1 : " + m_pathname);

    addCommands(
        // Add Commands here:
        //@formatter:off
    new AutoWait(1),
    new ParallelDeadlineGroup(
      new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished), 
      new AutoDrivePath(drivetrain, m_pathname, true)
    ),
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
