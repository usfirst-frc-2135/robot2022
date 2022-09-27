
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
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

  private Drivetrain dt;
  private String     m_pathname1 = AUTOConstants.k1BallLimelight_path1;
  private String     m_pathname2 = AUTOConstants.k1BallLimelight_path2;

  private boolean useLLValid( )
  {
    return dt.isLimelightValid(40, 25);
  }

  public Auto1BallLimelight(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    dt = drivetrain;
    setName("Auto1BallLimelight");

    DataLogManager.log("Auto1BallLimelight pathname 1 : " + m_pathname1);
    DataLogManager.log("Auto1BallLimelight pathname 2 : " + m_pathname2);

    addCommands(
        // Add Commands here:

        //@formatter:off
        new PrintCommand("AUTO: Use programmable delay from dashboard before starting"),
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
            new AutoDrivePath(drivetrain, m_pathname1, true)
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
          this::useLLValid  
        ), 

        new PrintCommand("AUTO: Run the second path off the tarmac"), 
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
