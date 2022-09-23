
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.AUTOConstants;

/**
 *
 */
public class AutoDrive extends SequentialCommandGroup
{
  private String m_pathname = AUTOConstants.kDrive_path;

  public AutoDrive(Drivetrain drivetrain, Intake intake)
  {
    setName("AutoDrive");

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
