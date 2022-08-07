// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.frc2135.RobotConfig;

/**
 *
 */
public class Vision extends SubsystemBase
{
  // Camera Limelight streaming states
  private final int    STANDARD      = 0;
  private final int    PIP_MAIN      = 1;
  private final int    PIP_SECONDARY = 2;

  // Limelight LED mode states
  private final int    LED_CUR_MODE  = 0;
  private final int    LED_OFF       = 1;
  private final int    LED_BLINK     = 2;
  private final int    LED_ON        = 3;

  private NetworkTable table;

  private double       targetHorizAngle; // Horizontal Offset from Crosshair to Target (-27 to 27 degrees)
  private double       targetVertAngle;  // Vertical Offset from Crosshair to Target (-20.5 to 20.5 degrees)
  private double       targetArea;       // Target Area (0% of image to 100% of image)
  private double       targetSkew;       // Target Skew or rotation (-90 degrees to 0 degrees)
  private boolean      targetValid;      // Target Valid or not

  // Variables in inches to calculate limelight distance
  private double       distance1     = 48;    // x position in inches for first reference point
  private double       vertOffset1   = 0.42;  // y reading in degrees for first reference point
  private double       distance2     = 60;    // x position in inches for second reference point
  private double       vertOffset2   = -4.85; // y reading in degrees for second reference point
  private double       distLL;                // calculated distance in inches for the current y value

  // Creates a MedianFilter with a window size of 5 samples
  MedianFilter         yfilter       = new MedianFilter(5); // filter y values to remove outliers

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
    setLEDMode(LED_ON);

    // Read these values from config file
    RobotConfig config = RobotConfig.getInstance( );
    distance1 = config.getValueAsDouble("VI_distance1", 48.0);
    distance2 = config.getValueAsDouble("VI_distance2", 60.0);
    vertOffset1 = config.getValueAsDouble("VI_vertOffset1", 0.42);
    vertOffset2 = config.getValueAsDouble("VI_vertOffset2", -4.85);

    // Put all the needed widgets on the dashboard
    SmartDashboard.putNumber("VI_distance1", distance1);
    SmartDashboard.putNumber("VI_distance2", distance2);
    SmartDashboard.putNumber("VI_vertOffset1", vertOffset1);
    SmartDashboard.putNumber("VI_vertOffset2", vertOffset2);

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
      targetHorizAngle = SmartDashboard.getNumber("VI_OVERRIDE_TX", 0.0);
      targetVertAngle = SmartDashboard.getNumber("VI_OVERRIDE_TY", 0.0);
      targetArea = SmartDashboard.getNumber("VI_OVERRIDE_TA", 0.0);
      targetSkew = SmartDashboard.getNumber("VI_OVERRIDE_TS", 0.0);
      targetValid = SmartDashboard.getBoolean("VI_OVERRIDE_TV", true);
    }
    else
    {
      targetHorizAngle = table.getEntry("tx").getDouble(0.0);
      targetVertAngle = yfilter.calculate(table.getEntry("ty").getDouble(0.0));
      targetArea = table.getEntry("ta").getDouble(0.0);
      targetSkew = table.getEntry("ts").getDouble(0.0);
      targetValid = table.getEntry("tv").getBoolean(false);
    }

    distLL = calculateDist(targetVertAngle);

    SmartDashboard.putNumber("VI_horizAngle", targetHorizAngle);
    SmartDashboard.putNumber("VI_vertAngle", targetVertAngle);
    SmartDashboard.putNumber("VI_area", targetArea);
    SmartDashboard.putNumber("VI_skew", targetSkew);
    SmartDashboard.putBoolean("VI_valid", targetValid);

    SmartDashboard.putNumber("VI_distLL", distLL);
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

    setLEDMode(LED_OFF);
    setCameraDisplay(PIP_SECONDARY);

    syncStateFromDashboard( );
  }

  public double getHorizOffsetDeg( )
  {
    return targetHorizAngle;
  }

  public double getVertOffsetDeg( )
  {
    return targetVertAngle;
  }

  public double getTargetArea( )
  {
    return targetArea;
  }

  public double getTargetSkew( )
  {
    return targetSkew;
  }

  public boolean getTargetValid( )
  {
    return targetValid;
  }

  public double getgetDistLimelight( )
  {
    return distLL;
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
    double slope = (distance2 - distance1) / (vertOffset2 - vertOffset1);
    double offset = distance1 - slope * vertOffset1;

    SmartDashboard.putNumber("VI_Slope", slope);

    return (slope * vertAngle) + offset;
  }

  private void syncStateFromDashboard( )
  {
    distance1 = SmartDashboard.getNumber("VI_distance1", distance1);
    distance2 = SmartDashboard.getNumber("VI_distance2", distance2);
    vertOffset1 = SmartDashboard.getNumber("VI_vertOffset1", vertOffset1);
    vertOffset2 = SmartDashboard.getNumber("VI_vertOffset2", vertOffset2);
  }
}
