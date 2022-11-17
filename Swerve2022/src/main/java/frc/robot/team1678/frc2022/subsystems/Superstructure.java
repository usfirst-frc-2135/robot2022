package frc.robot.team1678.frc2022.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.team1678.frc2022.Constants;
import frc.robot.team1678.frc2022.controlboard.ControlBoard;
import frc.robot.team1678.frc2022.drivers.Pigeon;
import frc.robot.team1678.frc2022.logger.LogStorage;
import frc.robot.team1678.frc2022.logger.LoggingSystem;
import frc.robot.team1678.frc2022.loops.ILooper;
import frc.robot.team1678.frc2022.loops.Loop;
import frc.robot.team1678.frc2022.subsystems.LEDs.State;
import frc.robot.team254.lib.geometry.Pose2d;
import frc.robot.team254.lib.util.Util;
import frc.robot.team254.lib.vision.AimingParameters;

public class Superstructure extends Subsystem {

    // superstructure instance
    private static Superstructure mInstance;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    /*** REQUIRED INSTANCES ***/
    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final Swerve mSwerve = Swerve.getInstance();
    private final LEDs mLEDs = LEDs.getInstance();
    private final Pigeon mPigeon = Pigeon.getInstance();

    // timer for reversing the intake and then stopping it once we have two correct
    // cargo
    Timer mIntakeRejectTimer = new Timer();
    // timer for asserting ball position
    Timer mAssertBallPositionTimer = new Timer();

    // PeriodicIO instance and paired csv writer
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    /*** CONTAINER FOR SUPERSTRUCTURE ACTIONS AND GOALS ***/
    public static class PeriodicIO {
        // INPUTS
        // (superstructure actions)
        private boolean INTAKE = false; // run the intake to pick up cargo
        private boolean REVERSE = false; // reverse the intake and singulator
        private boolean REJECT = false; // have the intake reject cargo
        private boolean PREP = false; // spin up and aim with shooting setpoints
        private boolean SHOOT = false; // shoot cargo
        private boolean FENDER = false; // shoot cargo from up against the hub
        private boolean SPIT = false; // spit cargo from shooter at low velocity

        // time measurements
        public double timestamp;
        public double dt;

        // OUTPUTS
        // (superstructure goals/setpoints)
        private double real_shooter = 0.0;
        private double real_hood = 0.0;
    }

    /* Setpoint Tracker Variables */
    public int mBallCount = 0; // number of balls in robot

    // shooting system setpoints
    public double mShooterSetpoint = 1000.0;
    public double mHoodSetpoint = 20.0;
    private double mHoodAngleAdjustment = 0.0; // on the fly shot angle adjustments
    private boolean mResetHoodAngleAdjustment = false; // reset hood angle adjustment

    // intake / eject locking status
    private boolean mIntakeReject = false; // reject third ball from entering intake
    private boolean mIntakeOverride = false; // backup override for intake locking logic
    private boolean mForceEject = false; // manually run the ejector
    private boolean mDisableEjecting = false; // backup override for ejecting logic
    private boolean mSlowEject = false; // eject at lower velocity for aimed ejection during auto

    // climb mode tracker variables
    private boolean mClimbMode = false; // if we are in climb mode (lock out other functions when climbing)
    private boolean mOpenLoopClimbControlMode = false; // open loop climber control for manual jogging
    private boolean mAutoTraversalClimb = false; // if we are running auto-traverse
    private boolean mAutoHighBarClimb = false; // if we are running auto-high
    private int mClimbStep = 0; // step of auto-climb we are currently on
    private double mRoll = 0.0; // roll of the robot

    // fender shot constants
    private final double kFenderVelocity = 2200;
    private final double kFenderAngle = 14.0;

    // spit shot constants
    private final double kSpitVelocity = 1000;
    private final double kSpitAngle = 20.0;

    // aiming parameter vars
    private Optional<AimingParameters> real_aiming_params_ = Optional.empty();
    private int mTrackId = -1;
    private double mTargetAngle = 0.0;
    private double mCorrectedDistanceToTarget = 0.0;

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                final double start = Timer.getFPGATimestamp();

                if (!mClimbMode) {
                    updateBallCounter();
                    updateSpitState();
                    updateShootingSetpoints();
                }

                setGoals();
                updateRumble();

                // send log data
                SendLog();

                final double end = Timer.getFPGATimestamp();
                mPeriodicIO.dt = end - start;
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    /*** SETTERS FOR SUPERSTRUCTURE ACTIONS OUTSIDE OPERATOR INPUT ***/
    public void setWantIntake(boolean intake) {
        mPeriodicIO.INTAKE = intake;

        // set other intake actions to false if this action is true
        if (mPeriodicIO.INTAKE) {
            mPeriodicIO.REVERSE = false;
            mPeriodicIO.REJECT = false;
        }
    }

    public void setWantReverse(boolean reverse) {
        mPeriodicIO.REVERSE = reverse;

        // set other intake actions to false if this action is true
        if (mPeriodicIO.REVERSE) {
            mPeriodicIO.INTAKE = false;
            mPeriodicIO.REJECT = false;
        }
    }

    public void setWantReject(boolean reject) {
        mPeriodicIO.REJECT = reject;

        // set other intake actions to false if this action is true
        if (mPeriodicIO.REJECT) {
            mPeriodicIO.INTAKE = false;
            mPeriodicIO.REVERSE = false;
        }
    }

    public void setWantIntakeNone() {
        mPeriodicIO.INTAKE = false;
        mPeriodicIO.REVERSE = false;
        mPeriodicIO.REJECT = false;
    }

    public void setWantEject(boolean eject, boolean slow_eject) {
        mForceEject = eject;
        mSlowEject = slow_eject;
    }

    public void setSlowEject(boolean slow_eject) {
        mSlowEject = slow_eject;
    }

    public void setEjectDisable(boolean enable) {
        mDisableEjecting = enable;
    }

    public void setWantPrep(boolean wants_prep) {
        mPeriodicIO.PREP = wants_prep;
    }

    public void setWantShoot(boolean wants_shoot) {
        mPeriodicIO.SHOOT = wants_shoot;
    }

    public void setShootingParameters(double flywheel, double hood) {
        mShooterSetpoint = flywheel;
        mHoodSetpoint = hood;
    }

    /***
     * CONTAINER FOR OPERATOR COMMANDS CALLING SUPERSTRUCTURE ACTIONS
     * 
     * Intaking
     * - hold right trigger to intake
     * - hold left trigger to manual eject
     * 
     * Shooting
     * - press A to prep for shot (spin up)
     * - press Y to shoot once ready
     * - press B to toggle fender shot with set params
     * - press X to toggle spit shot with set params
     * 
     * Manual Hood Adjustment
     * - Use dpad to manually adjust hood with offset
     * --> 0 to move hood up
     * --> 180 to move hood down
     * - press START button to reset adjustment
     * 
     * Other Manual Sets
     * - press dpad left (POV 270) to toggle force intake
     * - press dpad right (POV 90) to toggle disabling the ejector
     * - hold left bumper to eject balls manuallyo
     * 
     * Climb Controls
     * - press left bumper, right bumper, left trigger, right trigger to enter climb
     * mode
     * - press center "back" and "start" buttons to exit climb mode
     * - press down on left joystick to toggle open loop control of climber arms
     * - press down on right joystick to zero position on climber motors
     * 
     * - press A to prep for climb and extend right arm
     * - press B to only climb to mid bar
     * - press Y to complete full automated climb to traversal bar
     * 
     * - DPad down (POV 180) to climb mid bar and extend for high bar
     * - DPad right (POV 90) to climb high bar and extend for traversal bar
     * - DPad up (POV 0) to climb traversal bar
     * 
     */
    public void updateOperatorCommands() {
        {
            /*** NORMAL TELEOP CONTROLS ***/

            // disable toggle for intake locking
            if (mControlBoard.getDisableIntakeLogic()) {
                mIntakeOverride = !mIntakeOverride;
            }

            if (mControlBoard.getIntake()) {

                // lock intake control and start a rejection sequence if when intaking a third ball
                if ((indexerFull() || stopIntaking()) && !mIntakeReject) {
                    mIntakeReject = true;
                }

                // reverse intake for X seconds to ensure third ball has left system
                if (mIntakeReject && !mIntakeOverride) {
                    setWantReject(true);
                } else { // if we aren't rejecting
                    setWantIntake(true);
                }
            } else {
                mIntakeRejectTimer.reset();
                if (mControlBoard.getReject()) {
                    setWantReverse(true);
                } else {
                    setWantIntakeNone();
                }
            }

            // toggle ejecting to disable if necessary
            if (mControlBoard.getDisableColorLogic()) {
                mDisableEjecting = !mDisableEjecting;
            }

            // if we want to manual eject
            mForceEject = mControlBoard.getManualEject();

            // control shooting
            if (mControlBoard.getShoot()) {
                mPeriodicIO.SHOOT = !mPeriodicIO.SHOOT;

                // reset intake actions
                setWantIntakeNone();
            }

            // spin up to shoot if we aren't already
            if ((mPeriodicIO.SHOOT || mPeriodicIO.SPIT) && !mPeriodicIO.PREP) {
                mPeriodicIO.PREP = true;
            }

            // control prep
            if (mControlBoard.getPrep()) {
                mPeriodicIO.PREP = !mPeriodicIO.PREP;
            }

            // control fender shot
            if (mControlBoard.getFender()) {
                mPeriodicIO.FENDER = !mPeriodicIO.FENDER;
            }

            // non-toggle one ball spit shot
            if (mControlBoard.getSpit()) {
                mPeriodicIO.SPIT = true;
            }

            // control for adding manual hood adjustment
            switch (mControlBoard.getHoodManualAdjustment()) {
                case 1:
                    mHoodAngleAdjustment += 1;
                    break;
                case -1:
                    mHoodAngleAdjustment += -1;
                    break;
                case 0:
                    mHoodAngleAdjustment += 0;
                    break;
            }
            // reset manual hood adjustment if necessary
            if (mControlBoard.getResetHoodAdjust()) {
                mResetHoodAngleAdjustment = true;
            }
        }
      }

    /*** UPDATE BALL COUNTER FOR INDEXING STATUS ***/
    public void updateBallCounter() {
    }

    /***
     * UPDATE SPIT STATE 
     * Controls sequence to spit out one ball at a time
     **/
    public void updateSpitState() {
        // when two balls are in the indexer:
        mPeriodicIO.SPIT = false;
    }

    /*** RUMBLE OPERATOR CONTROLLERS WHILE SHOOTING ***/
    public void updateRumble() {
        if (!mClimbMode) {
            mControlBoard.setOperatorRumble(mPeriodicIO.SHOOT);
            mControlBoard.setDriverRumble(mPeriodicIO.SHOOT);
        } else {
            mControlBoard.setOperatorRumble(false);
            mControlBoard.setDriverRumble(false);
        }
    }

    /*** UPDATE SHOOTER AND HOOD GOALS FROM DISTANCE ***/
    public synchronized void updateShootingSetpoints() {
        if (mPeriodicIO.SPIT) {
            mShooterSetpoint = kSpitVelocity;
            mHoodSetpoint = kSpitAngle;
        } else if (mPeriodicIO.FENDER) {
            mShooterSetpoint = kFenderVelocity;
            mHoodSetpoint = kFenderAngle;
        } else if (real_aiming_params_.isPresent()) {
        }

    }

    /***
     * UPDATE SUBSYSTEM STATES + SETPOINTS AND SET GOALS
     * 
     * 1. updates wanted actions for intake and indexer subsystems based on
     * requested superstructure action
     * 2. updates shooter and hood setpoint goals from tracked vars
     * 3. set subsystem states and shooting setpoints within subsystems
     * 
     */
    public void setGoals() {
        /* Update subsystem wanted actions and setpoints */

        // reset hood angle adjustment if called
        if (mResetHoodAngleAdjustment) {
            mHoodAngleAdjustment = 0.0;
            mResetHoodAngleAdjustment = false;
        }
        // update hood setpoint
        mPeriodicIO.real_hood = mHoodSetpoint + mHoodAngleAdjustment;

        // update shooter setpoint
        if (mPeriodicIO.PREP) {
            mPeriodicIO.real_shooter = mShooterSetpoint;
        } else {
            mPeriodicIO.real_shooter = 0.0;
        }

        // update intake and indexer actions
        if (mPeriodicIO.SPIT) {
        } else if (mPeriodicIO.SHOOT) {
        } else {
        }
    }

    /*** UPDATE STATUS LEDS ON ROBOT ***/
    public void updateLEDs() {
        if (mLEDs.getUsingSmartdash()) {
            return;
        }

        State topState = State.OFF;
        State bottomState = State.OFF;

        if (hasEmergency) {
            topState = State.EMERGENCY;
            bottomState = State.EMERGENCY;
        } else {
            if (!mClimbMode) {
                if (getBallCount() == 2) {
                    bottomState = State.SOLID_GREEN;
                } else if (getBallCount() == 1) {
                    bottomState = State.SOLID_CYAN;
                } else {
                    bottomState = State.SOLID_ORANGE;
                }
                if (getWantsSpit()) {
                    topState = State.SOLID_ORANGE;
                } else if (getWantsFender()) {
                    topState = State.SOLID_CYAN;
                } else if (mPeriodicIO.SHOOT) {
                    topState = State.FLASHING_PINK;
                } else if (isAimed()) {
                    topState = State.FLASHING_GREEN;
                } else if (hasTarget()) {
                    topState = State.SOLID_PURPLE;
                } else {
                    topState = State.SOLID_ORANGE;
                }
            } else {
                if (mOpenLoopClimbControlMode) {
                    topState = State.SOLID_YELLOW;
                    bottomState = State.SOLID_YELLOW;
                } else if (mAutoTraversalClimb) {
                    topState = State.FLASHING_ORANGE;
                    bottomState = State.FLASHING_ORANGE;
                } else if (mAutoHighBarClimb) {
                    topState = State.FLASHING_CYAN;
                    bottomState = State.FLASHING_CYAN;
                } else {
                    topState = State.SOLID_PINK;
                    bottomState = State.SOLID_PINK;
                }
            }
        }

        mLEDs.applyStates(topState, bottomState);
    }

    // get vision align delta from goal
    public double getVisionAlignGoal() {
        return mTargetAngle;
    }

    // stop intaking if we have two of the correct cargo
    public boolean stopIntaking() {
        return (true);
    }

    // get number of correct cargo in indexer
    public double getBallCount() {
        return mBallCount;
    }

    // ball at back beam break and top beam break
    public boolean indexerFull() {
        return (mBallCount == 2);
    }

    // check if our flywheel is spun up to the correct velocity
    public boolean isSpunUp() {
        return true;
    }

    // check if our limelight sees a vision target
    public boolean hasTarget() {
        return true;
    }

    // checked if we are vision aligned to the target within an acceptable horiz. error
    public boolean isAimed() {
        return true;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void stop() {
        mPeriodicIO.INTAKE = false;
        mPeriodicIO.REVERSE = false;
        mPeriodicIO.REJECT = false;
        mPeriodicIO.PREP = false;
        mPeriodicIO.SHOOT = false;
        mPeriodicIO.FENDER = false;
        mPeriodicIO.SPIT = false;
        mForceEject = false;
    }

    /* Initial states for superstructure for teleop */
    public void setInitialTeleopStates() {
        mPeriodicIO.INTAKE = false;
        mPeriodicIO.REVERSE = false;
        mPeriodicIO.REJECT = false;
        mPeriodicIO.PREP = true; // stay spun up
        mPeriodicIO.SHOOT = false;
        mPeriodicIO.FENDER = false;
        mPeriodicIO.SPIT = false;

        System.out.println("Set initial teleop states!");
    }

    /* Superstructure getters for action and goal statuses */
    // get actions
    public boolean getIntaking() {
        return mPeriodicIO.INTAKE;
    }

    public boolean getReversing() {
        return mPeriodicIO.REVERSE;
    }

    public boolean getRejecting() {
        return mPeriodicIO.REJECT;
    }

    public boolean getEjecting() {
        return false;
    }

    public boolean getPrepping() {
        return mPeriodicIO.PREP;
    }

    public boolean getShooting() {
        return mPeriodicIO.SHOOT;
    }

    public boolean getWantsFender() {
        return mPeriodicIO.FENDER;
    }

    public boolean getWantsSpit() {
        return mPeriodicIO.SPIT;
    }

    public boolean getEjectDisabled() {
        return mDisableEjecting;
    }

    public boolean getIntakeOverride() {
        return mIntakeOverride;
    }

    // get other statuses
    public int getClimbStep() {
        return mClimbStep;
    }

    public boolean getInClimbMode() {
        return mClimbMode;
    }

    public boolean isOpenLoopClimbControl() {
        return mOpenLoopClimbControlMode;
    }

    public boolean isAutoClimb() {
        return mAutoTraversalClimb || mAutoHighBarClimb;
    }

    // get goals
    public String getIntakeGoal() {
        return "goal";
    }

    public double getShooterGoal() {
        return mPeriodicIO.real_shooter;
    }

    public double getHoodGoal() {
        return mPeriodicIO.real_hood;
    }

    // included to continue logging while disabled
    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mRoll = mPigeon.getRoll().getDegrees();
    }

    // logger
    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "SUPERSTRUCTURE_LOGS.csv");
    }

    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("timestamp");
        headers.add("dt");
        headers.add("INTAKE");
        headers.add("REVERSE");
        headers.add("REJECT");
        headers.add("EJECT");
        headers.add("PREP");
        headers.add("SHOOT");
        headers.add("FENDER");
        headers.add("SPIT");
        headers.add("real_shooter");
        headers.add("real_hood");
        headers.add("gyro roll");

        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(mPeriodicIO.timestamp);
        items.add(mPeriodicIO.dt);
        items.add(mPeriodicIO.INTAKE ? 1.0 : 0.0);
        items.add(mPeriodicIO.REVERSE ? 1.0 : 0.0);
        items.add(mPeriodicIO.REJECT ? 1.0 : 0.0);
        items.add(mPeriodicIO.PREP ? 1.0 : 0.0);
        items.add(mPeriodicIO.SHOOT ? 1.0 : 0.0);
        items.add(mPeriodicIO.FENDER ? 1.0 : 0.0);
        items.add(mPeriodicIO.SPIT ? 1.0 : 0.0);
        items.add(mPeriodicIO.real_shooter);
        items.add(mPeriodicIO.real_hood);
        items.add(mPigeon.getRoll().getDegrees());

        // send data to logging storage
        mStorage.addData(items);
    }

}
