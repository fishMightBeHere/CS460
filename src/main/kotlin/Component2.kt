import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.random.Random

class Component2 {
    companion object {
        fun random_rotation_matrix(naive: Boolean): Matrix {
            var rot = Matrix(3, 3)
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
                val theta = 2 * Math.PI * Random.nextDouble()
                val phi = 2 * Math.PI * Random.nextDouble()
                val z = Random.nextDouble()

                val v = Matrix(3, 1)
                v[0, 0] = cos(phi) * sqrt(z)
                v[1, 0] = sin(phi) * sqrt(z)
                v[2, 0] = sqrt(1 - z)

                val r = Matrix(3, 3)
                r[0] = doubleArrayOf(cos(theta), sin(theta), 0.0)
                r[1] = doubleArrayOf(-sin(theta), cos(theta), 0.0)
                r[2] = doubleArrayOf(0.0, 0.0, 1.0)

                rot = ((LinearAlgebra.multiply(v,LinearAlgebra.transpose(v))*2.0) - LinearAlgebra.identity(3))*r
            }

            return rot
        }
    }
}