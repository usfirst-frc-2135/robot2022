
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
public class Auto1BallLimelight extends SequentialCommandGroup
{
  public Auto1BallLimelight(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    setName("Auto1BallLimelight");

    String m_pathname1 = "fenderToOffTarmac";
    String m_pathname2 = "shootingPosToOffTarmac";

    DataLogManager.log("Auto1BallLimelight pathname 1 : " + m_pathname1);
    DataLogManager.log("Auto1BallLimelight pathname 2 : " + m_pathname2);

    addCommands(
        // Add Commands here:
        //@formatter:off
    new AutoWait(1), 

    // new ParallelDeadlineGroup(
    //   new IntakeDeploy(intake, true), 
    //   new AutoStop(drivetrain)
    // ),


    new ParallelDeadlineGroup(
      new ParallelDeadlineGroup(
        new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished), 
        new AutoDrivePath(drivetrain, m_pathname1, true)
        ), 
      new ScoringPrime(shooter, vision)
    ), 
    new PrintCommand("Run limelight shooting routine for 3rd ball"), 
    new ConditionalCommand(
      new AutoDriveLimelightShoot(drivetrain, intake, fConv, tConv, shooter, vision), 
      new ParallelDeadlineGroup(
        new ScoringStop(intake, fConv, tConv, shooter, vision), 
        new AutoStop(drivetrain)
      ), 
      drivetrain::useLLValid  //TODO: replace with a method to give a BooleanSupplier parameters
    ), 
    new ParallelDeadlineGroup(
      new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished), 
      new AutoDrivePath(drivetrain, m_pathname2, false),
      new ScoringStop(intake, fConv, tConv, shooter, vision)
    )


    //@formatter:on

    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
