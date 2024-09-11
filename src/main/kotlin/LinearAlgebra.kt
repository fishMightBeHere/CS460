import kotlin.math.pow

data class Matrix(val m:Int, val n:Int) {
    val matrix:Array<DoubleArray> = Array(m) { DoubleArray(n) }

    override fun toString(): String {
        return matrix.contentDeepToString()
    }

    operator fun get(row: Int) : DoubleArray {
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
}

class LinearAlgebra {
    companion object {
        fun rowMultiply(matrix: Matrix, row: Int, k: Double): Matrix {
            matrix[row].forEachIndexed { index, _ -> matrix[row, index] *= k }
            return matrix
        }

        fun rowAdd(matrix: Matrix, row1: Int, row2: Int, k: Double) : Matrix{
            matrix[row1].forEachIndexed {index, _ -> matrix[row1,index] += k * matrix[row2,index]}
            return matrix
        }

        fun rowSubtract(matrix: Matrix, row1: Int, row2: Int, k: Double): Matrix{
            matrix[row1].forEachIndexed {index, _ -> matrix[row1,index] -= k * matrix[row2,index]}
            return matrix
        }

        fun rowSwap(matrix: Matrix, row1: Int, row2: Int): Matrix {
            val r = matrix[row2]
            matrix[row2] = matrix[row1]
            matrix[row1] = r
            return matrix
        }

        fun determinant(matrix: Matrix) : Double {
            require(matrix.m == matrix.n) {"Non square matrix encountered, matrix is ${matrix.m} by ${matrix.n}"}
            return determinant(matrix,0,0)
        }

        fun minor(matrix: Matrix, i: Int, j: Int) : Matrix {
            require(matrix.m == matrix.n) {"Non square matrix encountered, matrix is ${matrix.m} by ${matrix.n}"}
            val minor:Matrix = Matrix(matrix.m-1,matrix.m-1)
            var im = 0
            var jm = 0
            for (il in 0 until matrix.m) {
                for (jl in 0 until matrix.m) {
                    if (il != i && jl != j) {
                        minor[im,jm] = matrix[il,jl]
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

        private fun determinant(matrix:Matrix, i: Int, j: Int):Double {
            // det A = sum (-1)^ij aij det(Aij)
            if (matrix.m == 2) {
                return matrix[0,0]*matrix[1,1] - matrix[0,1]*matrix[1,0]
            }
            return (-1.0).pow((i + j).toDouble()) * matrix[i,j] * determinant(minor(matrix,i,j), i+1, j)
        }

//        fun luDecomposition(matrix:Matrix): Pair<Matrix,Matrix> {
//            require(matrix.m == matrix.n) {"Non square matrix encountered, matrix is ${matrix.m} by ${matrix.n}"}
//
//
//
//        }

        fun multiply(a:Matrix, b:Matrix):Matrix {
            require(a.n == b.m) {"matrix dimensions do not match, a is ${a.m} x ${a.n}, b is ${b.m} x ${b.n}"}
            val ab:Matrix = Matrix(a.m, b.n)

            
            return ab
        }
    }
}

fun main() {
    val matrix:Matrix = Matrix(3,3)

    matrix[0,0] = 1.0
    matrix[1,1] = 2.0
    matrix[2,2] = 3.0

    println(matrix)

    LinearAlgebra.rowSwap(matrix, 0,1)
    println(matrix)
}
