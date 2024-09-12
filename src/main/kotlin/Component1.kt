class Component1{
    fun check_S0n(m:Matrix, epsilon:Double) : Boolean {
        // if matrix is part of S0n, it must have the property that the determinant equals 1, this assumes epsilon is error for floating point math
        return LinearAlgebra.determinant(m) >= 1-epsilon || LinearAlgebra.determinant(m) <= 1+epsilon;
    }

    fun check_quaternion(v:Matrix, epsilon:Double) : Boolean {
        // idk what a quaternion is
        return false;
    }

    fun check_SEn(m:Matrix, epsilon:Double) {
        
    }
}