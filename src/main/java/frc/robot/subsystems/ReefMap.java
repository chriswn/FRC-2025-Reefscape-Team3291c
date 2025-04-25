package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.Map;

/**
 * Tracks the Reef faces and levels scored, publishes to dashboard, and reads manual overrides.
 */
public class ReefMap {
<<<<<<< HEAD
  public enum Level {L1, L2, L3} // 0, 1, 2 corresponds to levels 1, 2, 3
=======
  public enum Level { L1, L2, L3, L4 }        // ← Added L4

>>>>>>> 85c5ae3c73aa630ae5af5d3411259d32da4df784
  public static class FaceState {
    public boolean l1 = false;
    public boolean l2 = false;
    public boolean l3 = false;
    public boolean l4 = false;                // ← Track 4th level
  }

  private final Map<Integer, FaceState> faceStates = new HashMap<>();
  private final SendableChooser<Integer> overrideChooser = new SendableChooser<>();

  public ReefMap() {
    // Initialize 6 reef faces: IDs 0-5
    for (int i = 0; i < 6; i++) {
      faceStates.put(i, new FaceState());
      // Publish all four levels to SmartDashboard
      for (Level lvl : Level.values()) {
        SmartDashboard.putBoolean(key(i, lvl), false);
      }
      overrideChooser.addOption("Face " + i, i);
    }
    overrideChooser.setDefaultOption("None", -1);
    SmartDashboard.putData("Reef Override Face", overrideChooser);
  }

  private String key(int face, Level level) {
    return String.format("Reef/Face%d_%s", face, level.name());
  }

  /** Mark a given face and level as scored. */
  public void markScored(int face, Level level) {
    FaceState fs = faceStates.get(face);
    if (fs == null) return;
    switch (level) {
      case L1: fs.l1 = true; break;
      case L2: fs.l2 = true; break;
      case L3: fs.l3 = true; break;
      case L4: fs.l4 = true; break;         // ← Handle L4
    }
    SmartDashboard.putBoolean(key(face, level), true);
  }

  public boolean hasUnscoredTargets() {
    for (FaceState fs : faceStates.values()) {
      if (!fs.l1 || !fs.l2 || !fs.l3 || !fs.l4) {
        return true;
      }
    }
    return false;
  }

  /** Reset all faces (e.g., at match start). */
  public void reset() {
    for (Map.Entry<Integer, FaceState> e : faceStates.entrySet()) {
      e.getValue().l1 = e.getValue().l2 = e.getValue().l3 = e.getValue().l4 = false;
      for (Level lvl : Level.values()) {
        SmartDashboard.putBoolean(key(e.getKey(), lvl), false);
      }
    }
    overrideChooser.setDefaultOption("None", -1);
    overrideChooser.setSelected(-1);       // ← also clear the selection
  }

  /**
   * Returns the manually selected face override, or -1 if none.
   */
  public int getOverrideFace() {
    Integer sel = overrideChooser.getSelected();
    return sel == null ? -1 : sel;
  }

  /**
   * Choose the next scoring target: either override or first available.
   * Returns an array [face, levelOrdinal], or null if all done.
   */
  public int[] getNextTarget() {
    int override = getOverrideFace();
    if (override >= 0 && override < 6) {
      FaceState fs = faceStates.get(override);
      for (Level lvl : Level.values()) {
        boolean done = switch (lvl) {
          case L1 -> fs.l1;
          case L2 -> fs.l2;
          case L3 -> fs.l3;
          case L4 -> fs.l4;
        };
        if (!done) {
          return new int[]{override, lvl.ordinal()};
        }
      }
    }
    // no valid override, pick first free
    for (int i = 0; i < 6; i++) {
      FaceState fs = faceStates.get(i);
      for (Level lvl : Level.values()) {
        boolean done = switch (lvl) {
          case L1 -> fs.l1;
          case L2 -> fs.l2;
          case L3 -> fs.l3;
          case L4 -> fs.l4;
        };
        if (!done) {
          return new int[]{i, lvl.ordinal()};
        }
      }
    }
    return null;
  }
}
