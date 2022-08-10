package frc.robot.frc2135;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotConfig
{
  private static final int    DUMMY_DEFAULT_INT    = Integer.MAX_VALUE;
  private static final float  DUMMY_DEFAULT_FLOAT  = Float.MAX_VALUE;
  private static final double DUMMY_DEFAULT_DOUBLE = Double.MAX_VALUE;

  private static RobotConfig  instance             = null;

  private void RobotConfig( )
  {
    // Log loadConfig Start
    loadConfig( );
    // Log loadConfig Finished
    // Log dumpConfig Start
    dumpConfig( );
    // Log dumpConfig Finished
  }

  public static RobotConfig getInstance( )
  {
    if (instance == null)
      instance = new RobotConfig( );

    return instance;
  }

  // Private methods

  private void trimWhitespace(String line)
  {

  }

  // Public methods

  public String getConfigFilename( )
  {
    // Initialize with the absolute path to the home directory
    String deployDirectory = Filesystem.getDeployDirectory( ).toPath( ).toString( );
    deployDirectory.concat("/");

    // Default the host name of the RoboRIO
    String nameBuf = "roborio-2135";

    // Adjust name if on real robot from hostname
    if (RobotBase.isReal( ))
      ;

    String filename = nameBuf.substring(8, 4);
    filename.concat("_configuration.txt");

    return deployDirectory.concat(filename);
  }

  public boolean loadConfig( )
  {
    String filename = getConfigFilename( );
    // TODO: open the file
    // if not successful
    // print error message
    // else
    // clear map
    // read config file into a map until eof
    //

    return true;
  }

  public void dumpConfig( )
  {

  }

  public String getValueAsString(String key, String defaultValue)
  {
    return defaultValue;
  }

  public int getValueAsInt(String key, int defaultValue)
  {
    return defaultValue;
  }

  public boolean getValueAsBool(String key, boolean defaultValue)
  {
    return defaultValue;
  }

  public float getValueAsFloat(String key, float value, float defaultValue)
  {
    return defaultValue;
  }

  public double getValueAsDouble(String key, double defaultValue)
  {
    return defaultValue;
  }
}
