class ArmSystem (val arms: List<Bot>, localPose: MutableList<Double> = mutableListOf()) {
    fun update() {
        arms.forEach {it.update()}
    }

    fun teleport(vararg poses:Double) {
        var globalPose = 0.0
        for ((arm, pose) in arms.zip(poses.toList())) {
            arm.teleport(Pair(Matrix(0.0,0.0),pose))
        }
    }
}