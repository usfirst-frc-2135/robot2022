
// ROBOTBUILDER TYPE: WaitCommand.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class AutoWait extends WaitCommand
{
  private int        m_timerNum;
  private Drivetrain m_drivetrain;
  private double     m_waitTime;

  public AutoWait(Drivetrain drivetrain, int timerNum)
  {
    super(timerNum);
    m_timerNum = timerNum;
    m_drivetrain = drivetrain;

    // m_subsystem = subsystem;
    // addRequirements(m_subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize( )
  {
    super.initialize( );
    if (m_timerNum == 1)
      m_waitTime = SmartDashboard.getNumber("AUTO_WaitTime1", 0.0);

    if (m_timerNum == 2)
      m_waitTime = SmartDashboard.getNumber("AUTO_WaitTime2", 0.0);

    m_timer.reset( );
    m_timer.start( );
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute( )
  {
    m_drivetrain.moveStop( );
  }

  public boolean IsFinished( )
  {
    return m_timer.hasElapsed(m_waitTime);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted)
  {
    super.end(interrupted);
    m_timer.stop( );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
