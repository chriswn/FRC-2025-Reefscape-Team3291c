package frc.robot.subsystems;

<<<<<<< HEAD
import frc.robot.Constants;
=======
import java.util.Optional;
>>>>>>> 85c5ae3c73aa630ae5af5d3411259d32da4df784
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class ScoringTargetManager {
  private final ReefMap reefMap = new ReefMap();
  private ScoringTarget currentTarget;

<<<<<<< HEAD
    /**
     * Reset at match start.
     */
    public void reset() {
        currentTarget = null;
        reefMap.reset(); // Optionally reset ReefMap if needed
=======
  /** Reset at match start. */
  public void reset() {
    currentTarget = null;
    reefMap.reset();                 // ← reset the per-face state too!
    SmartDashboard.putString("Current Target", "None");
  }

  /** Call a new target explicitly. */
  public void callTarget(ScoringTarget newTarget) {
    currentTarget = newTarget;
    SmartDashboard.putString("Current Target", newTarget.toString());
  }

  /** Returns the currently called target, if any. */
  public Optional<ScoringTarget> getCurrentTarget() {
    return Optional.ofNullable(currentTarget);
  }

  /** Determines next available target, or returns null if all done. */
  public ScoringTarget getNextTarget() {
    int[] next = reefMap.getNextTarget();
    if (next == null) {
      SmartDashboard.putString("Next Available Target", "All scored ✅");
      return null;
>>>>>>> 85c5ae3c73aa630ae5af5d3411259d32da4df784
    }
    // next[1] is 0–3; add 1 to get 1–4
    ScoringTarget t = new ScoringTarget(next[0], next[1] + 1);
    SmartDashboard.putString("Next Available Target", t.toString());
    return t;
  }

<<<<<<< HEAD
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
        int[] next = reefMap.getNextTarget(); // from your new ReefMap
        if (next == null) {
            SmartDashboard.putString("Next Available Target", "All scored ✅");
            return null;
        }

        // reefMap returns [face, levelOrdinal], where levelOrdinal is 0-2 for L1–L3
        // So we need to add 1 to levelOrdinal to get the actual level (1-3)
        ScoringTarget nextTarget = new ScoringTarget(next[0], next[1] + 1);  // Fix level mapping
        SmartDashboard.putString("Next Available Target", nextTarget.toString());
        return nextTarget;
    }

    /**
     * Check if there are unscored targets.
     */
    public boolean hasUnscoredTargets() {
        return reefMap.hasUnscoredTargets();
    }

    /**
     * Mark a target as scored and update internal state.
     */
    public void markScored(ScoringTarget target) {
        ReefMap.Level levelEnum = ReefMap.Level.values()[target.getLevel() - 1]; // because level = 1,2,3
        reefMap.markScored(target.getFace(), levelEnum);
        currentTarget = null; // clear after scoring
        SmartDashboard.putString("Current Target", "None");
    }
=======
  public boolean hasUnscoredTargets() {
    return reefMap.hasUnscoredTargets();
  }

  /** Mark the current target as scored and clear it. */
  public void markScored(ScoringTarget target) {
    ReefMap.Level lvl = ReefMap.Level.values()[target.getLevel() - 1];
    reefMap.markScored(target.getFace(), lvl);
    currentTarget = null;
    SmartDashboard.putString("Current Target", "None");
  }
>>>>>>> 85c5ae3c73aa630ae5af5d3411259d32da4df784
}
