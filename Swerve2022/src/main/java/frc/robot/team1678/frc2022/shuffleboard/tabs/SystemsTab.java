package frc.robot.team1678.frc2022.shuffleboard.tabs;

import frc.robot.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import frc.robot.team1678.frc2022.subsystems.Superstructure;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SystemsTab extends ShuffleboardTabBase {

	private Superstructure mSuperstructure = Superstructure.getInstance();

	private NetworkTableEntry mLimelightHasTarget;
	private NetworkTableEntry mLimelightLatency;
	private NetworkTableEntry mLimelightTX;
	private NetworkTableEntry mLimelightTY;

	private NetworkTableEntry mBottomBreak;
	private NetworkTableEntry mTopBreak;

	private NetworkTableEntry mClimbStep;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("System Check");

		mLimelightHasTarget = mTab
				.add("Limelight Has Target", false)
				.withSize(2, 1)
				.withPosition(0, 0)
				.getEntry();
		mLimelightLatency = mTab
				.add("Limelight Latency", 0.0)
				.withSize(2, 1)
				.withPosition(2, 0)
				.getEntry();
		mLimelightTX = mTab
				.add("tx", 0.0)
				.withSize(1, 1)
				.withPosition(4, 0)
				.getEntry();
		mLimelightTY = mTab
				.add("ty", 0.0)
				.withSize(1, 1)
				.withPosition(5, 0)
				.getEntry();
	}

	@Override
	public void update() {
	}

}
