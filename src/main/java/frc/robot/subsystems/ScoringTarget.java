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

<<<<<<< HEAD
    /**
     * Look up the AprilTag ID for this target using your Constants mapping.
     * Assumes you have defined in Constants.Vision:
     *   public static final int[][] TAG_IDS = new int[6][3];
     */
    public int getTagId() {
         // sanity-check the face index
    if (face < 0 || face >= Constants.Vision.TAG_IDS.length) {
        throw new IllegalArgumentException("Face index out of range: " + face);
    }
    
    // sanity-check the level
    if (level < 1 || level > 3) {
        throw new IllegalArgumentException("Level out of range: " + level);
    }
    
    // level-1 because the array is 0-based
    return Constants.Vision.TAG_IDS[face][level - 1];
    }

    @Override
    public String toString() {
        return String.format("ScoringTarget{face=%d, level=%d, tagId=%d}", 
            face, level, getTagId());
    }
}
=======
  @Override
  public String toString() {
    return String.format(
      "ScoringTarget{face=%d (tag %d), level=L%d}",
      face, getTagId(), level
    );
  }
}
    
>>>>>>> 85c5ae3c73aa630ae5af5d3411259d32da4df784
