// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VIConsts;
import frc.robot.frc2135.RobotConfig;

/**
 *
 */
public class Vision extends SubsystemBase
{
  private NetworkTable table;

  private double       m_targetHorizAngle; // Horizontal Offset from Crosshair to Target (-27 to 27 degrees)
  private double       m_targetVertAngle;  // Vertical Offset from Crosshair to Target (-20.5 to 20.5 degrees)
  private double       m_targetArea;       // Target Area (0% of image to 100% of image)
  private double       m_targetSkew;       // Target Skew or rotation (-90 degrees to 0 degrees)
  private boolean      m_targetValid;      // Target Valid or not

  // Variables in inches to calculate limelight distance
  private double       m_distance1   = 48;    // x position in inches for first reference point
  private double       m_vertOffset1 = 0.42;  // y reading in degrees for first reference point
  private double       m_distance2   = 60;    // x position in inches for second reference point
  private double       m_vertOffset2 = -4.85; // y reading in degrees for second reference point
  private double       m_distLL;                // calculated distance in inches for the current y value

  // Creates a MedianFilter with a window size of 5 samples
  MedianFilter         m_yfilter     = new MedianFilter(5); // filter y values to remove outliers

  /**
   *
   */
  public Vision( )
  {
    setName("Vision");
    setSubsystem("Vision");

    // Get the Network table reference once for all methods
    NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    table = inst.getTable("limelight");

    // Set camera and LED display
    setLEDMode(VIConsts.LED_ON);

    // Read these values from config file
    RobotConfig config = RobotConfig.getInstance( );
    m_distance1 = config.getValueAsDouble("VI_distance1", 48.0);
    m_distance2 = config.getValueAsDouble("VI_distance2", 60.0);
    m_vertOffset1 = config.getValueAsDouble("VI_vertOffset1", 0.42);
    m_vertOffset2 = config.getValueAsDouble("VI_vertOffset2", -4.85);

    // Put all the needed widgets on the dashboard
    SmartDashboard.putNumber("VI_distance1", m_distance1);
    SmartDashboard.putNumber("VI_distance2", m_distance2);
    SmartDashboard.putNumber("VI_vertOffset1", m_vertOffset1);
    SmartDashboard.putNumber("VI_vertOffset2", m_vertOffset2);

    SmartDashboard.setDefaultBoolean("VI_OVERRIDE", false);
    SmartDashboard.putNumber("VI_OVERRIDE_TX", 0.0);
    SmartDashboard.putNumber("VI_OVERRIDE_TY", 0.0);
    SmartDashboard.putNumber("VI_OVERRIDE_TA", 0.0);
    SmartDashboard.putNumber("VI_OVERRIDE_TS", 0.0);
    SmartDashboard.putBoolean("VI_OVERRIDE_TV", false);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    if (SmartDashboard.getBoolean("VI_OVERRIDE", false))
    {
      // Allow the limelight to be bypassed by entries from the dashboard
      m_targetHorizAngle = SmartDashboard.getNumber("VI_OVERRIDE_TX", 0.0);
      m_targetVertAngle = SmartDashboard.getNumber("VI_OVERRIDE_TY", 0.0);
      m_targetArea = SmartDashboard.getNumber("VI_OVERRIDE_TA", 0.0);
      m_targetSkew = SmartDashboard.getNumber("VI_OVERRIDE_TS", 0.0);
      m_targetValid = SmartDashboard.getBoolean("VI_OVERRIDE_TV", true);
    }
    else
    {
      m_targetHorizAngle = table.getEntry("tx").getDouble(0.0);
      m_targetVertAngle = m_yfilter.calculate(table.getEntry("ty").getDouble(0.0));
      m_targetArea = table.getEntry("ta").getDouble(0.0);
      m_targetSkew = table.getEntry("ts").getDouble(0.0);
      m_targetValid = table.getEntry("tv").getBoolean(false);
    }

    m_distLL = calculateDist(m_targetVertAngle);

    SmartDashboard.putNumber("VI_horizAngle", m_targetHorizAngle);
    SmartDashboard.putNumber("VI_vertAngle", m_targetVertAngle);
    SmartDashboard.putNumber("VI_area", m_targetArea);
    SmartDashboard.putNumber("VI_skew", m_targetSkew);
    SmartDashboard.putBoolean("VI_valid", m_targetValid);

    SmartDashboard.putNumber("VI_distLL", m_distLL);
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");

    setLEDMode(VIConsts.LED_OFF);
    setCameraDisplay(VIConsts.PIP_SECONDARY);

    syncStateFromDashboard( );
  }

  public double getHorizOffsetDeg( )
  {
    return m_targetHorizAngle;
  }

  public double getVertOffsetDeg( )
  {
    return m_targetVertAngle;
  }

  public double getTargetArea( )
  {
    return m_targetArea;
  }

  public double getTargetSkew( )
  {
    return m_targetSkew;
  }

  public boolean getTargetValid( )
  {
    return m_targetValid;
  }

  public double getDistLimelight( )
  {
    return m_distLL;
  }

  public void setLEDMode(int mode)
  {
    DataLogManager.log(getSubsystem( ) + ": setLedMode " + mode);

    NetworkTableEntry modeEntry = table.getEntry("ledMode");
    modeEntry.setValue(mode);
  }

  public int getLEDMode( )
  {
    int mode = (int) table.getEntry("ledMode").getNumber(0.0);

    DataLogManager.log(getSubsystem( ) + "getLedMode :" + mode);
    return mode;
  }

  public void setCameraDisplay(int stream)
  {
    DataLogManager.log(getSubsystem( ) + ": setCameraDisplay " + stream);

    NetworkTableEntry streamEntry = table.getEntry("stream");
    streamEntry.setValue(stream);
  }

  private double calculateDist(double vertAngle)
  {
    double slope = (m_distance2 - m_distance1) / (m_vertOffset2 - m_vertOffset1);
    double offset = m_distance1 - slope * m_vertOffset1;

    SmartDashboard.putNumber("VI_Slope", slope);

    return (slope * vertAngle) + offset;
  }

  public void syncStateFromDashboard( )
  {
    m_distance1 = SmartDashboard.getNumber("VI_distance1", m_distance1);
    m_distance2 = SmartDashboard.getNumber("VI_distance2", m_distance2);
    m_vertOffset1 = SmartDashboard.getNumber("VI_vertOffset1", m_vertOffset1);
    m_vertOffset2 = SmartDashboard.getNumber("VI_vertOffset2", m_vertOffset2);
  }
}
