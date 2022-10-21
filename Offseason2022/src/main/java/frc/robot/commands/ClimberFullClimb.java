
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CLConsts;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class ClimberFullClimb extends SequentialCommandGroup
{
  private double m_climbL2Time         = CLConsts.kClimbL2Time;
  private double m_rotateExtendL3Time  = CLConsts.kRotateExtendL3Time;
  private double m_rotateRetractL3Time = CLConsts.kRotateRetractL3Time;
  private double m_climbL3Time         = CLConsts.kClimbL3Time;
  private double m_rotateRetractL4Time = CLConsts.kRotateRetractL4Time;

  private Timer  m_timer               = new Timer( );

  public ClimberFullClimb(Climber climber, XboxController gamePad, Button button)
  {
    setName("ClimberFullClimb");

    SmartDashboard.putNumber("CL_climbL2Time", m_climbL2Time);
    SmartDashboard.putNumber("CL_rotateExtendL3Time", m_rotateExtendL3Time);
    SmartDashboard.putNumber("CL_rotateRetractL3Time", m_rotateRetractL3Time);
    SmartDashboard.putNumber("CL_climbL3Time", m_climbL3Time);
    SmartDashboard.putNumber("CL_rotateRetractL4Time", m_rotateRetractL4Time);

    m_timer.reset( );
    DataLogManager.log("Climb Timer Start: " + m_timer.get( ));

    m_timer.start( );

    addCommands(
        // Add Commands here:

        // @formatter:off
        new Climber2ClimbToL2(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_climbL2Time), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),
        new Climber3RotateToL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_rotateExtendL3Time), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),
        new Climber5RotateIntoL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_rotateRetractL3Time), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),
        new Climber6ClimbToL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_climbL3Time), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),

        // next rung climb!
        new Climber3RotateToL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_rotateExtendL3Time), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),
        new Climber5RotateIntoL3(climber),
        new ParallelRaceGroup(
            new WaitCommand(m_rotateRetractL4Time), 
            new ClimberTimerOverride(climber, gamePad, button)
        ),
        new Climber7ClimbToL4(climber)
        // @formatter:on 
    );
    m_timer.stop( );
    DataLogManager.log("Climb Timer End: " + m_timer.get( ));
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
