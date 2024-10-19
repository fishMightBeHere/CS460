import java.awt.Color
import kotlin.math.PI

class ArmSystem (val arms: List<Bot>) {

    fun teleport(vararg poses:Double) {
        require(poses.size == arms.size)
        for (i in arms.indices) {
            arms[i].teleport(Pair(Matrix(0.0,0.0),poses[i]))
            // update all child arms
            for (j in i+1..<arms.size) {
                arms[j].update()
            }
        }
    }

    fun draw() {
        arms.forEach(Bot::draw)
    }
}

fun main() {
    val l1 = Bot(
        Pair(Matrix(0.0,0.0),0.0),
        listOf(Pair(10.0,20.0),Pair(10.0,-20.0),Pair(-10.0,-20.0),Pair(-10.0,20.0)),
        endEffector = Pair(Matrix(0.0,20.0),0.0),
        botColor = Color.BLUE
    )
    val l2 = Bot(
        Pair(Matrix(0.0,0.0),0.0),
        listOf(Pair(10.0,30.0),Pair(10.0,0.0),Pair(-10.0,0.0),Pair(-10.0,30.0)),
        root = l1,
        botColor = Color.ORANGE
    )

    Drawer(500,500,200,200,1.0)
    val arms = ArmSystem(listOf(l1,l2))
    arms.teleport(0.0,PI/4)
    arms.draw()

    arms.teleport(PI/4,-PI/4)
    arms.draw()

}