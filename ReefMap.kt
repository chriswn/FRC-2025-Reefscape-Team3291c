data class ReefFaceState(
    var l1Scored: Boolean = false,
    var l2Scored: Boolean = false,
    var l3Scored: Boolean = false
) {
    fun isFullyScored(): Boolean = l1Scored && l2Scored && l3Scored
}

class ReefMap {
    // Assume 6 reef faces (3 per alliance side)
    private val reefFaces = MutableList(6) { ReefFaceState() }

    // Get state of a specific reef face
    fun getFaceState(index: Int): ReefFaceState? {
        return reefFaces.getOrNull(index)
    }

    // Mark a level as scored
    fun markScored(faceIndex: Int, level: Int) {
        reefFaces.getOrNull(faceIndex)?.let { face ->
            when (level) {
                1 -> face.l1Scored = true
                2 -> face.l2Scored = true
                3 -> face.l3Scored = true
            }
        }
    }

    // Reset all faces (for testing or match start)
    fun reset() {
        for (face in reefFaces) {
            face.l1Scored = false
            face.l2Scored = false
            face.l3Scored = false
        }
    }

    // Choose an optimal face + level (customize this with vision inputs + RP logic)
    fun getOptimalScoringTarget(): Pair<Int, Int>? {
        for ((i, face) in reefFaces.withIndex()) {
            if (!face.l1Scored) return i to 1
            if (!face.l2Scored) return i to 2
            if (!face.l3Scored) return i to 3
        }
        return null
    }

    // Manual override from dashboard
    fun setManualState(faceIndex: Int, l1: Boolean, l2: Boolean, l3: Boolean) {
        reefFaces.getOrNull(faceIndex)?.apply {
            l1Scored = l1
            l2Scored = l2
            l3Scored = l3
        }
    }

    fun getAllStates(): List<ReefFaceState> = reefFaces.toList()
}
