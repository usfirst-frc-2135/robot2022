
package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner
 * classes) wherever the constants are needed, to reduce verbosity.
 */
public class Constants
{
  public static final class Falcon500
  {
    public static final double kEncoderCPR = 2048; // CPR is 2048 from Falcon 500 Manual
  }

  /**
   * public static final class Drivetrain {
   * public static final int kLeftMotor1Port = 0;
   * public static final int kLeftMotor2Port = 1;
   * public static final int kRightMotor1Port = 2;
   * public static final int kRightMotor2Port = 3;
   * }
   */

  public static final class Drivetrain
  {

  }

  public static final class Intake
  {

  }

  public static final class FloorConveyor
  {

  }

  public static final class TowerConveyor
  {

  }

  public static final class Vision
  {
    // Camera Limelight streaming states
    public static final int STANDARD      = 0;
    public static final int PIP_MAIN      = 1;
    public static final int PIP_SECONDARY = 2;

    // Limelight LED mode states
    public static final int LED_CUR_MODE  = 0;
    public static final int LED_OFF       = 1;
    public static final int LED_BLINK     = 2;
    public static final int LED_ON        = 3;
  }

  public static final class Shooter
  {
    public static final double kFlywheelGearRatio = (18.0 / 12.0);
    public static final double kFlywheelCPR       = Falcon500.kEncoderCPR * kFlywheelGearRatio;
    public static final double kFlywheelPrimeRPM  = 200.0; // Target priming RPM

    public enum Mode
    {
      SHOOTER_REVERSE,  // Shooter runs in reverse direction to handle jams
      SHOOTER_STOP,     // Shooter is stopped
      SHOOTER_PRIME,    // Shooter ramped to an initial speed before shooting
      SHOOTER_LOWERHUB,   // Shooter at speed for low hub
      SHOOTER_UPPERHUB,  // Shooter at speed for high hub
    }
  }

  public static final class Climber
  {

  }

  public static final class LED
  {
    public enum LEDColor
    {
      LEDCOLOR_OFF, // enter
      LEDCOLOR_WHITE, // enter
      LEDCOLOR_RED, // ernter
      LEDCOLOR_ORANGE, // enter
      LEDCOLOR_YELLOW, // enter
      LEDCOLOR_GREEN, // enter
      LEDCOLOR_BLUE, // enter
      LEDCOLOR_PURPLE, // enter
      LEDCOLOR_DASH // enter
    }
  }
}
