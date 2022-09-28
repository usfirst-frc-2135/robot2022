
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
public class Auto1BallLimelight extends SequentialCommandGroup
{
  private Drivetrain m_drivetrain;

  private boolean isLLValid( )
  {
    return m_drivetrain.isLimelightValid(40, 25);
  }

  public Auto1BallLimelight(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    m_drivetrain = drivetrain;
    setName("Auto1BallLimelight");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("AUTO 1 BALL LL: Use programmable delay from dashboard"),
        new ParallelDeadlineGroup(
          new AutoWait(AutoTimer.TIMER1), 
          new AutoStop(drivetrain)
        ),

        // new PrintCommand("AUTO: Deploy intake if desired"),
        // new ParallelDeadlineGroup(
        //   new IntakeDeploy(intake, true), 
        //   new AutoStop(drivetrain)
        // ),
        
        new PrintCommand("AUTO: Run the first path while priming shooter"),
        new ParallelDeadlineGroup(
          new ParallelDeadlineGroup(
            new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished), 
            new AutoDrivePath(drivetrain, AUTOConstants.k1BallLimelight_path1, true)
            ), 
          new ScoringPrime(shooter, vision)
        ), 

        new PrintCommand("AUTO: Run limelight shooting routine for pre-loaded ball"), 
        new ConditionalCommand(
          new AutoDriveLimelightShoot(drivetrain, intake, fConv, tConv, shooter, vision), 
          new ParallelDeadlineGroup(
            new ScoringStop(intake, fConv, tConv, shooter, vision), 
            new AutoStop(drivetrain)
          ), 
          this::isLLValid
        ), 

        new PrintCommand("AUTO: Run the second path off the tarmac"), 
        new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished), 
          new AutoDrivePath(drivetrain, AUTOConstants.k1BallLimelight_path2, false),
          new ScoringStop(intake, fConv, tConv, shooter, vision)
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
