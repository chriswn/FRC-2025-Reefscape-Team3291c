package frc.robot.subsystems;

import frc.robot.Constants;

/**
 * Represents a scoring target on the reef: face index (0–5) and level (1–4).
 */
public class ScoringTarget {
  private final int face;
  private final int level; // 1–4

  public ScoringTarget(int face, int level) {
    if (face < 0 || face >= Constants.Vision.REEF_TAG_IDS.length) {
      throw new IllegalArgumentException("Face index out of range: " + face);
    }
    if (level < 1 || level > 4) {
      throw new IllegalArgumentException("Level out of range: " + level);
    }
    this.face = face;
    this.level = level;
  }

  public int getFace() { return face; }
  public int getLevel() { return level; }

  /** Returns the AprilTag ID for this face (levels share the same face tag). */
  public int getTagId() {
    return Constants.Vision.REEF_TAG_IDS[face];
  }

  @Override
  public String toString() {
    return String.format(
      "ScoringTarget{face=%d (tag %d), level=L%d}",
      face, getTagId(), level
    );
  }
}
    
