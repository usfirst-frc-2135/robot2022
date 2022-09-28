
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

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
import frc.robot.subsystems.Vision;

/**
 *
 */
public class Auto3BallRight extends SequentialCommandGroup
{
  Drivetrain m_drivetrain;

  private boolean isLLValid( )
  {
    return m_drivetrain.isLimelightValid(10, 15);
  }

  public Auto3BallRight(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    m_drivetrain = drivetrain;
    setName("Auto3BallRight");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("AUTO 3 BALL RIGHT: Use programmable delay from dashboard before starting"),
        new ParallelDeadlineGroup(
          new AutoWait(AutoTimer.TIMER1), 
          new AutoStop(drivetrain)
        ),
        
        new PrintCommand("AUTO: Deploy intake"),
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, true), 
          new AutoStop(drivetrain) 
        ),

        new PrintCommand("AUTO: Drive to a shooting position"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
          new AutoDrivePath(drivetrain, AUTOConstants.k3BallRight_path1, true),
          new ScoringPrime(shooter, vision) 
        ),

        new PrintCommand("AUTO: Shoot preloaded ball"),
        new ParallelDeadlineGroup (
          new ScoringActionUpperHub(intake, fConv, tConv, shooter, 1), 
          new AutoStop(drivetrain) 
        ),

        new PrintCommand("AUTO: Drive to 2nd ball and intake"),
        new ParallelDeadlineGroup(
            new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
            new AutoDrivePath(drivetrain, AUTOConstants.k3BallRight_path2, false),
            new IntakingAction(intake, fConv, tConv),
            new ScoringPrime(shooter, vision) 
          ),

        new PrintCommand("AUTO: Drive to a shooting position"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
          new AutoDrivePath(drivetrain, AUTOConstants.k3BallRight_path3, false),
          new IntakingAction(intake, fConv, tConv) 
        ),

        new PrintCommand("AUTO: Shoot 2nd ball"),
        new ParallelDeadlineGroup( 
          new ScoringActionUpperHub(intake, fConv, tConv, shooter, 2), 
          new AutoStop(drivetrain) 
        ),

        new PrintCommand("AUTO: Drive to 3rd ball"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
          new AutoDrivePath(drivetrain, AUTOConstants.k3BallRight_path4, false),
          new ScoringPrime(shooter, vision),
          new IntakingAction(intake, fConv, tConv) 
        ),

        new PrintCommand("AUTO: Drive to a shooting position"),
        new ParallelDeadlineGroup( 
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
          new AutoDrivePath(drivetrain, AUTOConstants.k3BallRight_path5, false),
          new IntakingAction(intake, fConv, tConv),
          new ScoringPrime(shooter, vision)
        ),

        new PrintCommand("AUTO: Run limelight shooting routine for 3rd ball"),
        new ConditionalCommand( 
          new AutoDriveLimelightShoot(drivetrain, intake, fConv, tConv, shooter, vision),
          new ScoringActionUpperHub(intake, fConv, tConv, shooter, 2),
          this::isLLValid
        ),

        // Drive towards human player/terminal
        // ParallelCommandGroup{
        //     ParallelDeadlineGroup{
        //         WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
        //         AutoDrivePath(m_pathname6.c_str(), false, drivetrain) },
        //     ScoringStop(intake, fConv, vConv, shooter) }
        // );

        new PrintCommand("AUTO: Stop shooting and driving"),
        new ParallelDeadlineGroup(
          new ScoringStop(intake, fConv, tConv, shooter, vision), 
          new AutoStop(drivetrain) 
        ),
        
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
