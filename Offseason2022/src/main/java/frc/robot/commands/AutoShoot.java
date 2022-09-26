
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AUTOConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;
import frc.robot.subsystems.Vision;

/**
 *
 */
public class AutoShoot extends SequentialCommandGroup
{
  private String m_pathname1 = AUTOConstants.kShoot_path;

  public AutoShoot(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter, Vision vision)
  {
    setName("AutoShoot");
    DataLogManager.log("AutoShoot pathname 1 : " + m_pathname1);

    addCommands(
        // Add Commands here:
        //@formatter:off
        new PrintCommand("AUTO: Deply intake at start"),
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, true), 
          new AutoStop(drivetrain)
        ),

        new PrintCommand("AUTO: Run the path and prime the shooter"),
        new ParallelCommandGroup(
          new ParallelDeadlineGroup(
            new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
            new AutoDrivePath(drivetrain, m_pathname1, true)
          ), 
          new ScoringPrime(shooter, vision)
        ),

        new PrintCommand("AUTO: Shoot the cargo"),
        new ParallelDeadlineGroup(
          new ScoringActionUpperHub(intake, fConv, tConv, shooter, 5.0), 
          new AutoStop(drivetrain)
        ),

        new PrintCommand("AUTO: Stop the shooter"),
        new ScoringStop(intake, fConv, tConv, shooter, vision)
        //@formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
