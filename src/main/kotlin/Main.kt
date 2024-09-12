import kotlin.math.cos
import kotlin.math.sin

fun main(args: Array<String>) {
    val m:Matrix = Matrix(2,2)
    m[0,0] = cos(3.14159/4)
    m[0,1] = -sin(3.14159/4)
    m[1,0] = sin(3.14159/4)
    m[1,1] = cos(3.14159/4)

    println(m)

    if(Component1.check_S0n(m,0.01)) println("yes")
    else println("no")
}