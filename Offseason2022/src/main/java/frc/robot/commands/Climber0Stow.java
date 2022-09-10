
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CLConsts.Height;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.Constants.INConsts.INMode;
import frc.robot.Constants.SHConsts.Mode;
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

    addCommands(new DriveSlowMode(drivetrain, false), new IntakeDeploy(intake, false), new IntakeRun(intake, INMode.INTAKE_STOP),
        new FloorConveyorRun(fConv, FCMode.FCONVEYOR_STOP), new TowerConveyorRun(tConv, TCMode.TCONVEYOR_STOP),
        new ShooterRun(shooter, Mode.SHOOTER_STOP),
        new ParallelDeadlineGroup(new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
            new ClimberMoveToHeight(climber, Height.STOW_HEIGHT)),
        new WaitCommand(1.5), new ParallelDeadlineGroup(new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
            new ClimberMoveToHeight(climber, Height.GATEHOOK_REST_HEIGHT)),
        new ClimberSetGatehook(climber, false));
    // Add Commands here:
    // Also add parallel commands using the
    //
    // addCommands(
    // new command1(argsN, subsystem),
    // parallel(
    // new command2(argsN, subsystem),
    // new command3(argsN, subsystem)
    // )
    // );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
