
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AUTOConstants;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class AutoPathSequence extends SequentialCommandGroup
{

  private String m_pathname1 = AUTOConstants.path1;
  private String m_pathname2 = AUTOConstants.path2;
  private String m_pathname3 = AUTOConstants.path3;

  public AutoPathSequence(Drivetrain drivetrain)
  {
    setName("AutoPathSequence");

    DataLogManager.log("AutoPath  path1 : " + m_pathname1);
    DataLogManager.log("AutoPath  path2 : " + m_pathname2);
    DataLogManager.log("AutoPath  path3 : " + m_pathname3);

    addCommands(
        // Add Commands here:

        //@formatter:off
        new PrintCommand("AUTO: Run first path"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
          new AutoDrivePath (drivetrain, AUTOConstants.path1, true)
        ),

        new PrintCommand("AUTO: Run second path"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
          new AutoDrivePath ( drivetrain, AUTOConstants.path2, false)
        ),

        new PrintCommand("AUTO: Hold in place"),
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
