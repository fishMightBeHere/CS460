import kotlin.math.cos
import kotlin.math.sin
import kotlin.random.Random

class Component2 {
    companion object {
        fun random_rotation(naive: Boolean): Matrix {
            val rot = Matrix(3, 3)
            if (naive) {
                val rx = Random.nextDouble(0.0, 2 * Math.PI)
                val ry = Random.nextDouble(0.0, 2 * Math.PI)
                val rz = Random.nextDouble(0.0, 2 * Math.PI)

                rot[0] = doubleArrayOf(
                    cos(ry) * cos(rz),
                    sin(rx) * sin(ry) * cos(rz) - cos(rx) * sin(rz),
                    cos(rx) * sin(ry) * cos(rz) + sin(rx) * sin(rz)
                )
                rot[1] = doubleArrayOf(
                    cos(ry) * sin(rz),
                    sin(rx) * sin(ry) * sin(rz) + cos(rx) * cos(rz),
                    cos(rx) * sin(ry) * sin(rz) - sin(rx) * cos(rz)
                )
                rot[2] = doubleArrayOf(-sin(ry), sin(rx) * cos(ry), cos(rx) * cos(ry))


            } else {

            }

            return rot
        }
    }
}