
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
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
import frc.robot.subsystems.Vision;

/**
 *
 */
public class Auto1Ball2OppLeft extends SequentialCommandGroup
{
  private boolean dummy( )
  {
    return SmartDashboard.getBoolean("AUTO_ShootOppBall", false);
  }

  public Auto1Ball2OppLeft(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    setName("AutoBall2OppLeft");

    // @formatter:off
    addCommands(
      new PrintCommand("AUTO 1 BALL 2 OPP LEFT - START"),
      // wait timer set in SmarDashboard new PrintCommand("WAIT"), new AutoWait(1), // Deploy Intake
      new PrintCommand("Deply intake"), 
      new ParallelDeadlineGroup(
       new IntakeDeploy(intake, true), 
       new AutoStop(drivetrain)
       ), 
      // Drive to a shooting position 
      new PrintCommand("drive to a shooting position"), 
      new ParallelDeadlineGroup(
        new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished
      ), 
      new AutoDrivePath(drivetrain, AUTOConstants.k1Ball2OppLeft_path2, true), 
      new ScoringPrime(shooter, vision)
      ), 
      // Shoot preloaded ball 
      new PrintCommand("Shoot preloaded ball"), 
      new ParallelDeadlineGroup(
        new ScoringActionUpperHub(intake, fConv, tConv, shooter, 1.0), 
        new AutoStop(drivetrain)
      ), 
      // Drive to 1st opponent's ball and intake 
      new PrintCommand("Drive to 1st opponent's ball and intake"), 
      new ParallelDeadlineGroup(
        new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished), 
        new AutoDrivePath(drivetrain, AUTOConstants.k1Ball2OppLeft_path2, false), 
        new IntakingAction(intake, fConv, tConv)
      ), 
      // Second wait timer set in SmartDashboard 
      new PrintCommand("WAIT"), 
      new AutoWait(AutoTimer.TIMER2), 
      // Drive to 2cnd opponent's ball and intake 
      new PrintCommand("Drive to 2cnd opponent's ball and intake"), 
      new ParallelDeadlineGroup(
        new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished), 
        new AutoDrivePath(drivetrain, AUTOConstants.k1Ball2OppLeft_path3, false), 
        new ScoringPrime(shooter, vision), 
        new IntakingAction(intake, fConv, tConv)
      ), 
      // stow intake new PrintCommand("Stow intake"), 
      new ParallelDeadlineGroup(
        new IntakeDeploy(intake, false), 
        new AutoStop(drivetrain)
      ),
      //PUT IN CONDITIONAL COMMAND: LITERALLY IF THIS RETURN THIS OR LIKE RETURN THE BOOLEAN 
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
        new PrintCommand("AUTO 3 BALL LEFT - END")
      )
    );
      // @formatter:on
  }

  @Override
  public final boolean runsWhenDisabled( )
  {
    return false;
  }
}
