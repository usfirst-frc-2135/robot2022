package frc.robot.team1678.frc2022.shuffleboard;

import java.util.ArrayList;
import java.util.List;

import frc.robot.team1678.frc2022.shuffleboard.tabs.LedTab;
import frc.robot.team1678.frc2022.shuffleboard.tabs.OperatorTab;
import frc.robot.team1678.frc2022.shuffleboard.tabs.SuperstructureTab;
import frc.robot.team1678.frc2022.shuffleboard.tabs.SwerveTab;
import frc.robot.team1678.frc2022.shuffleboard.tabs.SystemsTab;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleBoardInteractions {

    // Trims unneccesary tabs when in competition
    public final boolean mDebug = false;

    /* ShuffleBoardInteractions Instance */
    private static ShuffleBoardInteractions mInstance; 

    public static ShuffleBoardInteractions getInstance() {
        if (mInstance == null) {
            mInstance = new ShuffleBoardInteractions();
        }
        return mInstance;
    }

    private ArrayList<ShuffleboardTabBase> mTabs = new ArrayList<ShuffleboardTabBase>();

    private OperatorTab mOperatorTab;

    // instantiate subsystems, tabs, and widgets
    public ShuffleBoardInteractions() {
        mOperatorTab = new OperatorTab();
        mTabs.add(mOperatorTab);

        if (mDebug) {
            List<ShuffleboardTabBase> optionalTabs = List.of(
                new SwerveTab(),
                new SuperstructureTab(),
                new LedTab()
            );
            mTabs.addAll(optionalTabs);
        } else {
            mTabs.add(new SystemsTab());
        }

        for(ShuffleboardTabBase tab: mTabs) {
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : mTabs) {
            tab.update();
        }
    }

    public ShuffleboardTab getOperatorTab() {
        return mOperatorTab.getTab();
    }
}
 