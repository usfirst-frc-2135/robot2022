
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

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
public class Auto1Ball1OppRight extends SequentialCommandGroup
{
  private boolean autoSelector( )
  {
    return SmartDashboard.getBoolean("AUTO_ShootOppBall", false);
  }

  public Auto1Ball1OppRight(Drivetrain drivetrain, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Vision vision)
  {
    setName("Auto1Ball1OppRight");

    // get values as string part of code

    addCommands(new PrintCommand("AUTO 1 BALL 1 OPP RIGHT - START"),
    // @formatter:off   
    // Wait timer set in SmartDashboard
        new PrintCommand("WAIT"), 
        new AutoWait(AutoTimer.TIMER1),
        // deploy intake
        new PrintCommand("Deploy intake"), 
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, true), 
          new AutoStop(drivetrain)
        ),
        // drive to a shooting position
        new PrintCommand("drive to a shooitng position"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
          new AutoDrivePath(
            drivetrain, AUTOConstants.k1Ball1OppRight_path1, true), 
            new ScoringPrime(shooter, vision)
          ),
        // shoot preloaded ball
        new PrintCommand("shoot preloaded ball"),
        new ParallelDeadlineGroup(
          new ScoringActionUpperHub(intake, fConv, tConv, shooter, 1), 
          new AutoStop(drivetrain)
        ),
        // wait time set in SmartDashboard
        new PrintCommand("WAIT"), 
        new AutoWait(AutoTimer.TIMER2),
        // drive to opponent's ball and intake
        new PrintCommand("drive to opponent's ball and intake"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(drivetrain::driveWithPathFollowerIsFinished),
          new AutoDrivePath(drivetrain, AUTOConstants.k1Ball1OppRight_path2, false), 
          new IntakingAction(intake, fConv, tConv),
          new ScoringPrime(shooter, vision)
        ),
        new PrintCommand("Stow intake"), 
        new ParallelDeadlineGroup(
          new IntakeDeploy(intake, false), 
          new AutoStop(drivetrain)
        ),
    // shoot opponent's ball
        new ConditionalCommand(
          new ParallelDeadlineGroup(
            new ScoringActionLowerHub(intake, fConv, tConv, shooter, 2), 
            new AutoStop(drivetrain), 
            new PrintCommand("Shoot opponent's ball") 
          ), 
          new AutoStop(drivetrain),
          this::autoSelector
        ),
   //smartdashboard return get boolean shootOppBall 
        new PrintCommand("stop shooitng"), 
        new ParallelDeadlineGroup( 
          new ScoringStop(intake, fConv, tConv, shooter, vision), 
          new AutoStop(drivetrain) 
        ), 
        new PrintCommand("AUTO 1 BALL 1 OPP RIGHT - END")
    // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
