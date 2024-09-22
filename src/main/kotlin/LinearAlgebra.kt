import java.util.stream.IntStream
import kotlin.math.pow

data class Matrix(val m: Int, val n: Int) {

    constructor(vararg e:Double) : this(m=e.size,n=1) {
        e.forEachIndexed { index, d -> matrix[index][0] = d }
    }

    val matrix: Array<DoubleArray> = Array(m) { DoubleArray(n) }

    fun copy() : Matrix {
        val copy = Matrix(this.m,this.n)
        for (i in 0..<this.m) {
            for (j in 0..<this.n) {
                copy[i,j] = this[i,j]
            }
        }
        return copy
    }
    override fun toString(): String {
        return matrix.contentDeepToString()
    }

    operator fun get(row: Int): DoubleArray {
        return matrix[row]
    }

    operator fun set(index: Int, row: DoubleArray) {
        matrix[index] = row
    }

    operator fun get(row: Int, col: Int): Double {
        return matrix[row][col]
    }

    operator fun set(row: Int, col: Int, value: Double) {
        matrix[row][col] = value
    }

    fun getColumn(col: Int): DoubleArray {
        val c = DoubleArray(m)
        IntStream.range(0, m).forEach { i -> c[i] = matrix[i][col] }
        return c
    }
    //operator for matrix multiplication
    operator fun times(b:Matrix) : Matrix {
        return LinearAlgebra.multiply(this,b)
    }
    //operator for scalar matrix multiplication
    operator fun times(b:Double) : Matrix {
        val r = Matrix(this.m,this.n)
        for (i in 0..<this.m) {
            for (j in 0..<this.n) {
                r[i,j] = this[i,j] * b
            }
        }
        return r
    }

    operator fun minus(b: Matrix) : Matrix {
        require(this.m == b.m && this.n == b.n) {"matrix dimensions do not match: [${this.m} x ${this.n}] - [${b.m} x ${b.n}"}
        val r = this.copy()
        for (i in 0..<r.m) {
            for (j in 0..<r.n) {
                r[i,j] -= b[i,j]
            }
        }
        return r
    }

    operator fun plus(b: Matrix) :Matrix {
        require(this.m == b.m && this.n == b.n) {"matrix dimensions do not match: [${this.m} x ${this.n}] - [${b.m} x ${b.n}"}
        val r = this.copy()
        for (i in 0..<r.m) {
            for (j in 0..<r.n) {
                r[i,j] += b[i,j]
            }
        }
        return r
    }
}

// functions should not be side effect based
class LinearAlgebra {
    companion object {
        fun rowMultiply(matrix: Matrix, row: Int, k: Double): Matrix {
            val m = matrix.copy()
            m[row].forEachIndexed { index, _ -> m[row, index] *= k }
            return m
        }

        fun rowAdd(matrix: Matrix, row1: Int, row2: Int, k: Double): Matrix {
            val m = matrix.copy()
            m[row1].forEachIndexed { index, _ -> m[row1, index] += k * m[row2, index] }
            return m
        }

        fun rowSubtract(matrix: Matrix, row1: Int, row2: Int, k: Double): Matrix {
            val m = matrix.copy()
            m[row1].forEachIndexed { index, _ -> m[row1, index] -= k * m[row2, index] }
            return m
        }

        fun rowSwap(matrix: Matrix, row1: Int, row2: Int): Matrix {
            val m = matrix.copy()
            val r = m[row2]
            m[row2] = m[row1]
            m[row1] = r
            return m
        }

        fun determinant(matrix: Matrix): Double {
            require(matrix.m == matrix.n) { "Non square matrix encountered, matrix is ${matrix.m} by ${matrix.n}" }
            val m = matrix.copy()
            return determinant(m, 0, 0)
        }

        fun minor(matrix: Matrix, i: Int, j: Int): Matrix {
            require(matrix.m == matrix.n) { "Non square matrix encountered, matrix is ${matrix.m} by ${matrix.n}" }
            val minor: Matrix = Matrix(matrix.m - 1, matrix.m - 1)
            var im = 0
            var jm = 0
            for (il in 0..<matrix.m) {
                for (jl in 0..<matrix.m) {
                    if (il != i && jl != j) {
                        minor[im, jm] = matrix[il, jl]
                        jm++
                    }
                }
                jm = 0
                if (il != i) {
                    im++
                }
            }
            return minor
        }

        private fun determinant(matrix: Matrix, i: Int, j: Int): Double {
            // det A = sum (-1)^ij aij det(Aij)
            if (matrix.m == 2) {
                return matrix[0, 0] * matrix[1, 1] - matrix[0, 1] * matrix[1, 0]
            }
            return (-1.0).pow((i + j).toDouble()) * matrix[i, j] * determinant(minor(matrix, i, j), i + 1, j)
        }

        fun multiply(a: Matrix, b: Matrix): Matrix {
            fun dp(v1: DoubleArray, v2: DoubleArray): Double =
                IntStream.range(0, v1.size).mapToDouble { i -> v1[i] * v2[i] }.sum()

            require(a.n == b.m) { "matrix dimensions do not match, a is ${a.m} x ${a.n}, b is ${b.m} x ${b.n}" }
            val ab: Matrix = Matrix(a.m, b.n)
            for (i in 0..<ab.m) {
                for (j in 0..<ab.n) {
                    ab[i, j] = dp(a[i], b.getColumn(j))
                }
            }

            return ab
        }

        fun transpose(matrix: Matrix): Matrix {
            val t = Matrix(matrix.n, matrix.m)
            for (i in 0..<matrix.m) {
                for (j in 0..<matrix.n) {
                    t[j, i] = matrix[i, j]
                }
            }
            return t
        }

        fun identity(n:Int) : Matrix {
            val m = Matrix(n,n)
            IntStream.range(0,n).forEach { i-> m[i,i] = 1.0 }
            return m
        }
    }
}

fun main() {
    val matrix: Matrix = Matrix(2, 1)
    matrix[0,0] = 10.0
    matrix[1,0] = 20.0
    val m2 = Matrix(2,1)
    m2[0,0] = 5.0
    m2[1,0] = 0.0
    println(matrix+m2)


}
