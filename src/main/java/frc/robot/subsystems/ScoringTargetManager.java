package frc.robot.subsystems;

import java.util.Optional;
import frc.robot.Constants;
import frc.robot.subsystems.ReefMap;
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
          return null;
        }
        // reefMap returns [face, levelOrdinal], but levelOrdinal is 0-2 for L1-L3
        return new ScoringTarget(next[0], next[1] + 1);
    }
}
