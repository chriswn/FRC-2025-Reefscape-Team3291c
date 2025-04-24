package frc.robot.subsystems;

import java.util.Optional;
import frc.robot.Constants;
import frc.robot.subsystems.ReefMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Manages the current and next scoring targets.
 */
public class ScoringTargetManager {
    private ReefMap reefMap = new ReefMap(); // Initialize ReefMap instance
    private ScoringTarget currentTarget;

    /**
     * Reset at match start.
     */
    public void reset() {
        currentTarget = null;
    }

    /**
     * Call a new target explicitly.
     */
    public void callTarget(ScoringTarget newTarget) {
        this.currentTarget = newTarget;
        SmartDashboard.putString("Current Target", newTarget.toString());
        
    }

    /**
     * Returns the currently called target, if any.
     */
    public Optional<ScoringTarget> getCurrentTarget() {
        return Optional.ofNullable(currentTarget);
    }

    /**
     * Determines next available target based on simple priority: faces 0→5, levels 1→3.
     */
    public ScoringTarget getNextTarget() {
        // // Replace with your ReefMap logic if available
        // for (int face = 0; face < 6; face++) {
        //     for (int lvl = 1; lvl <= 3; lvl++) {
        //         // TODO: Skip if already scored via ReefMap
        //         return new ScoringTarget(face, lvl);
        //     }
        // }
        // return null;

        int[] next = reefMap.getNextTarget(); // from your new ReefMap
        if (next == null) {
            SmartDashboard.putString("Next Available Target", "All scored ✅");
          return null;
        }

        ScoringTarget nextTarget = new ScoringTarget(next[0], next[1] + 1);
        SmartDashboard.putString("Next Available Target", nextTarget.toString());

        
            // reefMap returns [face, levelOrdinal], where levelOrdinal is 0-2 for L1–L3
            // So we need to add 1 to levelOrdinal to get the actual level (1-3)
                // return new ScoringTarget(next[0], next[1] + 1);
        return nextTarget;
   
    }

    public void markScored(ScoringTarget target) {
        ReefMap.Level levelEnum = ReefMap.Level.values()[target.getLevel() - 1]; // because level = 1,2,3
        reefMap.markScored(target.getFace(), levelEnum);
        currentTarget = null; // clear after scoring
        SmartDashboard.putString("Current Target", "None");
    }
    
}
