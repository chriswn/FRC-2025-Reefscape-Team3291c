package frc.robot.subsystems;

import frc.robot.Constants;

/**
 * Represents a scoring target on the reef: face index (0-5) and level (1-3).
 */
public class ScoringTarget {
    private final int face;
    private final int level; // 1, 2, or 3

    /**
     * @param face 0–5, corresponds to reef face
     * @param level 1–3, corresponds to scoring level on that face
     */
    public ScoringTarget(int face, int level) {
        this.face = face;
        this.level = level;
    }

    public int getFace() {
        return face;
    }

    public int getLevel() {
        return level;
    }

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