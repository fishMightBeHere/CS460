import java.util.stream.IntStream

class Component1{
    companion object {

        fun check_S0n(m: Matrix, epsilon: Double): Boolean {
            // if matrix is part of S0n, it must have the property that the determinant equals 1, this assumes epsilon is error for floating point math
            return LinearAlgebra.determinant(m) >= 1 - epsilon || LinearAlgebra.determinant(m) <= 1 + epsilon;
        }

        fun check_quaternion(v: Matrix, epsilon: Double): Boolean {
            return v.m == 4;
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