import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.random.Random

class A1C1{
    companion object {

        fun check_S0n(m: Matrix, epsilon: Double): Boolean {
            // if matrix is part of S0n, it must have the property that the determinant equals 1, this assumes epsilon is error for floating point math
            return LinearAlgebra.determinant(m) >= 1 - epsilon || LinearAlgebra.determinant(m) <= 1 + epsilon;
        }

        fun check_quaternion(v: Matrix, epsilon: Double): Boolean {
            if (v.m != 4) return false
            return v[0,0]*v[0,0] + v[1,0]*v[1,0] + v[2,0]*v[2,0] + v[3,0]*v[3,0] <= 1 + epsilon || v[0,0]*v[0,0] + v[1,0]*v[1,0] + v[2,0]*v[2,0] + v[3,0]*v[3,0] >= 1 - epsilon
        }

        fun check_SEn(m: Matrix, epsilon: Double):Boolean{
            // seperate m into rotation and homogenous translation matrix, and verify both

            val rotation = LinearAlgebra.minor(m,m.m-1, m.n-1)
            if (!check_S0n(rotation, epsilon)) return false;
            for (i in 0..<m.n-1) {
                if (m[m.m-1,i] != 0.0) return false
            }
            return m[m.m-1,m.n-1] == 1.0
        }
    }
}

fun main() {
    println(A1C1.check_S0n(A1C2.random_rotation_matrix(false).also { i-> println(i) }, 0.01))
    println(A1C1.check_quaternion(A1C2.random_quaternion(false).also { i-> println(i) },0.01))

    val theta = 2* PI* Random.nextDouble()
    val m = Matrix(doubleArrayOf(cos(theta),-sin(theta),20.0), doubleArrayOf(sin(theta),cos(theta),15.0), doubleArrayOf(0.0,0.0,1.0))
    println(m)
    println(A1C1.check_SEn(m,0.01))



}