
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AUTOConstants;
import frc.robot.Constants.AUTOConstants.AutoTimer;
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

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("AUTO DRIVE: Use programmable delay from dashboard before starting"),
        new ParallelDeadlineGroup(
          new AutoWait(AutoTimer.TIMER1), 
          new AutoStop(drivetrain)
        ),

        new PrintCommand("AUTO: Drive a path off the tarmac"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished), 
          new AutoDrivePath(drivetrain, AUTOConstants.kDrive_path, true)
        ),

        new PrintCommand("AUTO: Sit still while feeding motors"),
        new AutoStop(drivetrain)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
