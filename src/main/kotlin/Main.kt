fun main(args: Array<String>) {
    val m:Matrix = Matrix(3,3)

    if (A1C1.check_S0n(A1C2.random_rotation_matrix(false).also { i->println(i) }, 0.01)) println("yes") else println("no")
}