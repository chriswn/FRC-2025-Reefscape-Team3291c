package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.Map;

/**
 * Tracks the Reef faces and levels scored, publishes to dashboard, and reads manual overrides.
 */
public class ReefMap {
  public enum Level {L1, L2, L3}
  public static class FaceState {
    public boolean l1 = false;
    public boolean l2 = false;
    public boolean l3 = false;
  }

  private final Map<Integer, FaceState> faceStates = new HashMap<>();
  private final SendableChooser<Integer> overrideChooser = new SendableChooser<>();

  public ReefMap() {
    // Initialize 6 reef faces: IDs 0-5
    for (int i = 0; i < 6; i++) {
      faceStates.put(i, new FaceState());
      SmartDashboard.putBoolean(key(i, Level.L1), false);
      SmartDashboard.putBoolean(key(i, Level.L2), false);
      SmartDashboard.putBoolean(key(i, Level.L3), false);
      overrideChooser.addOption("Face " + i, i);
    }
    overrideChooser.setDefaultOption("None", -1);
    SmartDashboard.putData("Reef Override Face", overrideChooser);
  }

  private String key(int face, Level level) {
    return String.format("Reef/Face%d_%s", face, level.name());
  }

  /**
   * Mark a given face and level as scored.
   */
  public void markScored(int face, Level level) {
    FaceState fs = faceStates.get(face);
    if (fs == null) return;
    switch(level) {
      case L1: fs.l1 = true; break;
      case L2: fs.l2 = true; break;
      case L3: fs.l3 = true; break;
    }
    SmartDashboard.putBoolean(key(face, level), true);
  }

  /**
   * Reset all faces (e.g., at match start).
   */
  public void reset() {
    for (Map.Entry<Integer, FaceState> e : faceStates.entrySet()) {
      e.getValue().l1 = e.getValue().l2 = e.getValue().l3 = false;
      SmartDashboard.putBoolean(key(e.getKey(), Level.L1), false);
      SmartDashboard.putBoolean(key(e.getKey(), Level.L2), false);
      SmartDashboard.putBoolean(key(e.getKey(), Level.L3), false);
    }
    overrideChooser.setDefaultOption("None", -1);
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
      if (!fs.l1) return new int[]{override, Level.L1.ordinal()};
      if (!fs.l2) return new int[]{override, Level.L2.ordinal()};
      if (!fs.l3) return new int[]{override, Level.L3.ordinal()};
    }
    // no valid override, pick first free
    for (int i = 0; i < 6; i++) {
      FaceState fs = faceStates.get(i);
      if (!fs.l1) return new int[]{i, Level.L1.ordinal()};
      if (!fs.l2) return new int[]{i, Level.L2.ordinal()};
      if (!fs.l3) return new int[]{i, Level.L3.ordinal()};
    }
    return null;
  }
}
