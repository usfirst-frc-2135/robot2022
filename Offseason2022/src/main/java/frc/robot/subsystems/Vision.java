
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
public class Vision extends SubsystemBase
{

  private double            targetHorizAngle;                   // Horizontal Offset from Crosshair to Target (-27
                                                                // degrees to 27 degrees)
  private double            targetVertAngle;                    // Vertical Offset from Crosshair to Target (-20.5
                                                                // degrees to 20.5 degrees)
  private double            targetArea;                         // Target Area (0% of image to 100% of image)
  private double            targetSkew;                         // Target Skew or rotation (-90 degrees to 0 degrees)
  private boolean           targetValid;                        // Target Valid or not

  private NetworkTableEntry modeEntry;
  private NetworkTableEntry streamEntry;
  private NetworkTable      table;

  // variables in inches to calculate limelight distance
  private double            slope;
  private double            distOffset;
  private double            distance1     = 48;
  private double            distance2     = 60;
  private double            vertOffset1   = 0.42;
  private double            vertOffset2   = -4.85;

  private double            distLight;

  // Creates a MedianFilter with a window size of 5 samples
  MedianFilter              yfilter       = new MedianFilter(5);

  // Camera Limelight streaming
  private final int         STANDARD      = 3;
  private final int         PIP_MAIN      = 1;
  private final int         PIP_SECONDARY = 2;

  // Limelight LED mode states
  private final int         LED_CUR_MODE  = 0;
  private final int         LED_OFF       = 1;
  private final int         LED_BLINK     = 2;
  private final int         LED_ON        = 3;

  /**
   *
   */
  public Vision( )
  {
    setName("Vision");

    setSubsystem("Vision");

    NetworkTableInstance inst = NetworkTableInstance.getDefault( );

    table = inst.getTable("limelight");
    setLEDMode(LED_ON);
    setCameraDisplay(PIP_SECONDARY);
    // Set camera and LED display

    SmartDashboard.setDefaultBoolean("VI_OVERRIDE", false);

    SmartDashboard.putNumber("VI_Distance1", distance1);
    SmartDashboard.putNumber("VI_Distance2", distance2);
    SmartDashboard.putNumber("VI_VertOffset1", vertOffset1);
    SmartDashboard.putNumber("VI_VertOffset2", vertOffset2);

  }

  double calculateDist( )
  {
    slope = (distance2 - distance1) / (vertOffset2 - vertOffset1);
    distOffset = distance1 - slope * vertOffset1;
    distLight = (slope * targetVertAngle) + distOffset;

    return distLight;
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    boolean smOverrideEnabled = SmartDashboard.getBoolean("VI_SOVERRIDE_ENABLED", false);
    if (smOverrideEnabled)
    {
      // During daytime hours we can use smartdashboard to bipass the limelight.
      // This will allow us to calibrate the shooter distance without relying
      // on lighting conditions or limelight tuning.
      targetHorizAngle = SmartDashboard.getNumber("VI_OVERRIDE_TX", 0.0);
      targetVertAngle = SmartDashboard.getNumber("VI_OVERRIDE_TY", 0.0);
      targetArea = SmartDashboard.getNumber("VI_OVERRIDE_TA", 0.0);
      targetSkew = SmartDashboard.getNumber("VI_OVERRIDE_SKEW", 0.0);
      targetValid = SmartDashboard.getBoolean("VI_OVERRIDE_TV", true);
    }
    else
    {
      // How to access NetworkTable or get values from it? NetworkTablesEntry or NetworkTables?

      targetHorizAngle = table.getEntry("tx").getDouble(0.0);
      targetVertAngle = yfilter.calculate(table.getEntry("ty").getDouble(0.0));
      targetArea = table.getEntry("ta").getDouble(0.0);
      targetSkew = table.getEntry("ts").getDouble(0.0);
      // TODO: remove median filter during port to java
      targetValid = table.getEntry("tv").getBoolean(false);
    }

    calculateDist( );

    SmartDashboard.putNumber("VI_HORIZ_ANGLE", targetHorizAngle);
    SmartDashboard.putNumber("VI_VERT_ANGLE", targetVertAngle);
    SmartDashboard.putNumber("VI_TARGET_AREA", targetArea);
    SmartDashboard.putNumber("VI_TARGET_SKEW", targetSkew);
    SmartDashboard.putBoolean("VI_TARGET_VALID", targetValid);
    SmartDashboard.putNumber("VI_Slope", slope);
    SmartDashboard.putNumber("VI_DistanceLimeLight", distLight);

  }

  void syncStateFromDashboard( )
  {
    distance1 = SmartDashboard.getNumber("VI_Distance1", distance1);
    distance2 = SmartDashboard.getNumber("VI_Distance2", distance2);
    vertOffset1 = SmartDashboard.getNumber("VI_VertOffset1", vertOffset1);
    vertOffset2 = SmartDashboard.getNumber("VI_VertOffset2", vertOffset2);
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  double doubleGetHorizOffsetDeg( )
  {
    return targetHorizAngle;
  }

  double getVertOffsetDeg( )
  {
    return targetVertAngle;
  }

  double getTargetArea( )
  {
    return targetArea;
  }

  double getTargetSkew( )
  {
    return targetSkew;
  }

  double getLimeLightDist( )
  {
    return distLight;
  }

  boolean getTargetValid( )
  {
    return targetValid;
  }

  void setLEDMode(int mode)
  {
    // TODO: find a putNumber replacer
    modeEntry = table.getEntry("ledMode");
    modeEntry.setValue(mode);
    // table.putNumber("ledMode", mode);

    // TODO: Replace with DataLogs
    // spdlog::info("VI SetLedMode : {}", mode);
  }

  int getLEDMode( )
  {
    int mode = LED_CUR_MODE;
    // TODO: find a replacement for getInt for an int
    mode = (int) table.getEntry("ledMode").getNumber(0.0);

    // TODO: Replace with DataLogs
    // spdlog::info("VI GetLedMode : {}", mode);
    return mode;
  }

  void setCameraDisplay(int stream)
  {
    streamEntry = table.getEntry("stream");
    modeEntry.setValue(stream);

    // TODO: Replace with DataLogs
    // spdlog::info("VI SetCameraDisplay : {}", stream);
  }

}
