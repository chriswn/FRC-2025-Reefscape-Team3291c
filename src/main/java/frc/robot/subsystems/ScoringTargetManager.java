package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScoringTargetManager {
  private final ReefMap reefMap = new ReefMap();
  private ScoringTarget currentTarget;

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
    }
    // next[1] is 0–3; add 1 to get 1–4
    ScoringTarget t = new ScoringTarget(next[0], next[1] + 1);
    SmartDashboard.putString("Next Available Target", t.toString());
    return t;
  }

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
}
