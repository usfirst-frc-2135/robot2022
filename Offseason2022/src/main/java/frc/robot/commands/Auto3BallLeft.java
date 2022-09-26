
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AUTOConstants;
import frc.robot.Constants.AUTOConstants.AutoTimer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;
import frc.robot.subsystems.Vision;;

/**
 *
 */
public class Auto3BallLeft extends SequentialCommandGroup
{
  private boolean dummy( )
  {
    return SmartDashboard.getBoolean("AUTO_ShootOppBall", false);
  }

  public Auto3BallLeft(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    // @formatter:off
    addCommands(
      new PrintCommand("AUTO 3 BALL LEFT - START"),
      //wait timer set in SmartDashbaord
      new AutoWait(AutoTimer.TIMER1),
      // Deploy intake
      new PrintCommand("Deploy intake"),
      new ParallelDeadlineGroup(
        new IntakeDeploy(intake, true), 
        new AutoStop(drivetrain) 
      ),
      // Drive to a shooting position
      new PrintCommand("Drive to a shooting position"),
      new ParallelDeadlineGroup (
        new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
        new AutoDrivePath(drivetrain, AUTOConstants.k3BallLeft_path1, true),
        new ScoringPrime(shooter, vision) 
      ),
      // Shoot preloaded ball
      new PrintCommand("Shoot preloaded ball"),
      new ParallelDeadlineGroup(
        new ScoringActionUpperHub(intake, fConv, tConv, shooter, 1), 
        new AutoStop(drivetrain) 
      ),
      // Drive to 2nd ball and intake
      new PrintCommand("Drive to 2nd ball and intake"),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
        new AutoDrivePath(drivetrain, AUTOConstants.k3BallLeft_path2, false),
        new IntakingAction(intake, fConv, tConv),
        new ScoringPrime(shooter, vision) 
      ),
      new PrintCommand("Drive to a shooting position"),
      // Drive to a shooting position
      new ParallelDeadlineGroup(
        new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
        new AutoDrivePath(drivetrain, AUTOConstants.k3BallLeft_path3, false),
        new IntakingAction(intake, fConv, tConv) 
      ),
      // Shoot 2nd ball
      new PrintCommand("Shoot 2nd ball"),
      new ParallelDeadlineGroup(
        new ScoringActionUpperHub(intake, fConv, tConv, shooter, 2), 
        new AutoStop(drivetrain) 
      ),
      // Drive to opponent's ball and intake
      new PrintCommand("Drive to opponent's ball and intake"),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
        new AutoDrivePath(drivetrain, AUTOConstants.k3BallLeft_path4, false),
        new ScoringPrime(shooter, vision),
        new IntakingAction(intake, fConv, tConv) 
      ),
      // Stow intake
      new PrintCommand("Stow intake"),
      new ParallelDeadlineGroup(
          new IntakeDeploy(intake, false), 
          new AutoStop(drivetrain) 
        ),
      // Shoot opponent's ball
      new ConditionalCommand(
        new ParallelDeadlineGroup( 
          new ScoringActionLowerHub(intake, fConv, tConv, shooter, 2),
          new AutoStop(drivetrain),
          new PrintCommand("Shoot opponent's ball")
        ),
        new AutoStop(drivetrain),
        this::dummy  
      ),
      // Stop shooting and driving
      new PrintCommand("Stop shooting"),
      new ParallelDeadlineGroup(
        new ScoringStop(intake, fConv, tConv, shooter, vision), 
        new AutoStop(drivetrain) 
      ),
      new PrintCommand("AUTO 3 BALL LEFT - END")

    // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
