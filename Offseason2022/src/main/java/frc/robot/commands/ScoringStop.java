
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.Constants.INConsts.INMode;
import frc.robot.Constants.SHConsts.SHMode;
import frc.robot.Constants.TCConsts.TCMode;
import frc.robot.subsystems.FloorConveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TowerConveyor;

/**
 *
 */
public class ScoringStop extends SequentialCommandGroup
{
  public ScoringStop(Intake intake, FloorConveyor fConv, TowerConveyor tConv, Shooter shooter)
  {
    setName("ScoringStop");

    addCommands(
    //@formatter:off
        // Add Commands here:
        new VisionOn(false), 
        new IntakeRun(intake,INMode.INTAKE_STOP), 
        new FloorConveyorRun(fConv, FCMode.FCONVEYOR_STOP),
        new TowerConveyorRun(tConv, TCMode.TCONVEYOR_STOP), 
        new ShooterRun(shooter, SHMode.SHOOTER_STOP));
    //@formatter:on

  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
