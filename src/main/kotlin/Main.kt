import kotlin.math.cos
import kotlin.math.sin

fun main(args: Array<String>) {
    val m:Matrix = Matrix(3,3)

    if (Component1.check_S0n(Component2.random_rotation_matrix(false).also { i->println(i) }, 0.01)) println("yes") else println("no")
}