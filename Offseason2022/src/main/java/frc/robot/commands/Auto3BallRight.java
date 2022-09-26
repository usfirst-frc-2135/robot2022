
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
public class Auto3BallRight extends SequentialCommandGroup
{
  /*
   * private boolean dummy(Drivetrain drive) { return SmartDashboard.getBoolean(key, defaultValue)
   * isLimelightValid(10, 15); }
   */
  private boolean dummy( )
  {
    return SmartDashboard.getBoolean("AUTO_ShootOppBall", false);
  }

  // @formatter:off
  public Auto3BallRight(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
  Vision vision)
  {
    addCommands(
      new PrintCommand("AUTO 3 BALL RIGHT - START"),
      // Wait timer set in SmartDasboard
      new AutoWait(AutoTimer.TIMER1),
      // Deploy intake
      new PrintCommand("Deploy intake"),
      new ParallelDeadlineGroup(
        new IntakeDeploy(intake, true), 
        new AutoStop(drivetrain) 
      ),
      // Drive to a shooting position
      new PrintCommand("Drive to a shooting position"),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
        new AutoDrivePath(drivetrain, AUTOConstants.k3BallRight_path1, true),
        new ScoringPrime(shooter, vision) 
      ),
      // Shoot preloaded ball
      new PrintCommand("Shoot preloaded ball"),
      new ParallelDeadlineGroup (
        new ScoringActionUpperHub(intake, fConv, tConv, shooter, 1), 
        new AutoStop(drivetrain) 
      ),
      // Drive to 2nd ball and intake
      new PrintCommand("Drive to 2nd ball and intake"),
      new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
          new AutoDrivePath(drivetrain, AUTOConstants.k3BallRight_path2, false),
          new IntakingAction(intake, fConv, tConv),
          new ScoringPrime(shooter, vision) 
        ),
      // Drive to a shooting position
      new PrintCommand("Drive to a shooting position"),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
        new AutoDrivePath(drivetrain, AUTOConstants.k3BallRight_path3, false),
        new IntakingAction(intake, fConv, tConv) 
      ),
      // Shoot 2nd ball
      new PrintCommand("Shoot 2nd ball"),
      new ParallelDeadlineGroup( 
        new ScoringActionUpperHub(intake, fConv, tConv, shooter, 2), 
        new AutoStop(drivetrain) 
      ),
      // Drive to 3rd ball
      new PrintCommand("Drive to 3rd ball"),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
        new AutoDrivePath(drivetrain, AUTOConstants.k3BallRight_path4, false),
        new ScoringPrime(shooter, vision),
        new IntakingAction(intake, fConv, tConv) 
      ),
      // Drive to a shooting position
      new PrintCommand("Drive to a shooting position"),
      new ParallelDeadlineGroup( 
        new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
        new AutoDrivePath(drivetrain, AUTOConstants.k3BallRight_path5, false),
        new IntakingAction(intake, fConv, tConv),
        new ScoringPrime(shooter, vision),
        new PrintCommand("Driving to shooting position group")
      ),
      // Run limelight shooting routine for 3rd ball
      new PrintCommand("Run limelight shooting routine for 3rd ball"),
      new ConditionalCommand( 
        new AutoDriveLimelightShoot(drivetrain, intake, fConv, tConv, shooter, vision),
        new ScoringActionUpperHub(intake, fConv, tConv, shooter, 2),
        this::dummy
        //this::dummy(drivetrain)
      ),
      // Drive towards human player/terminal
      // frc2::ParallelCommandGroup{
      //     frc2::ParallelDeadlineGroup{
      //         frc2::WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
      //         AutoDrivePath(m_pathname6.c_str(), false, drivetrain) },
      //     ScoringStop(intake, fConv, vConv, shooter) });
      // Stop shooting and driving
      new PrintCommand("Stop shooting and driving"),
      new ParallelDeadlineGroup(
        new ScoringStop(intake, fConv, tConv, shooter, vision), 
        new AutoStop(drivetrain) 
      ),
      new PrintCommand("AUTO 3 BALL RIGHT - END")
     // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
