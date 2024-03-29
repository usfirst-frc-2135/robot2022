
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CLConsts.CLHeight;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.Constants.INConsts.INMode;
import frc.robot.Constants.SHConsts.SHMode;
import frc.robot.Constants.TCConsts.TCMode;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;

/**
 *
 */
public class Climber0Stow extends SequentialCommandGroup
{
  public Climber0Stow(Climber climber, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Drivetrain drivetrain)
  {
    setName("Climber0Stow");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand("Climber --- Stow climber, reset all other subsystems ---"),
        new DriveSlowMode(drivetrain, false), 
        new IntakeDeploy(intake, false), 
        new IntakeRun(intake, INMode.INTAKE_STOP),
        new FloorConveyorRun(fConv, FCMode.FCONVEYOR_STOP), 
        new TowerConveyorRun(tConv, TCMode.TCONVEYOR_STOP),
        new ShooterRun(shooter, SHMode.SHOOTER_STOP),
        new ParallelDeadlineGroup(
            new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
            new ClimberMoveToHeight(climber, CLHeight.HEIGHT_STOW)
        ),

        new PrintCommand("Climber --- Relax climber to rest position ---"),
        new WaitCommand(1.5), 
        new ParallelDeadlineGroup(
            new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
            new ClimberMoveToHeight(climber, CLHeight.HEIGHT_GATEHOOK_REST)
        ),
        new ClimberSetGatehook(climber, false)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
