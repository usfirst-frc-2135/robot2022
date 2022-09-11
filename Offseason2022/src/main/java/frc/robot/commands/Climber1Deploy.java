
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class Climber1Deploy extends SequentialCommandGroup
{
  public Climber1Deploy(Climber climber, Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter,
      Drivetrain drivetrain)
  {
    setName("Climber1Deploy");

    addCommands(
        // Add Commands here:

        // @formatter:off
      new DriveSlowMode(drivetrain, true), 
      new IntakeDeploy(intake, false), 
      new IntakeRun(intake, INMode.INTAKE_STOP),
      new FloorConveyorRun(fConv, FCMode.FCONVEYOR_STOP), 
      new TowerConveyorRun(tConv, TCMode.TCONVEYOR_STOP),
      new ShooterRun(shooter, Mode.SHOOTER_STOP),
      new ParallelDeadlineGroup(
          new WaitUntilCommand(climber::moveClimberDistanceIsFinished),
          new ClimberMoveToHeight(climber, Height.EXTEND_L2_HEIGHT)), 
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
