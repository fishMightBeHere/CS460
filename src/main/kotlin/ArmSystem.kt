import java.awt.Color
import kotlin.math.PI

class ArmSystem (val arms: List<Bot>, localPose: MutableList<Double> = mutableListOf()) {
    fun update() {
        arms.forEach {it.update()}
    }

    fun teleport(vararg poses:Double) {
        var globalPose = 0.0
        for ((arm, pose) in arms.zip(poses.toList())) {
            arm.teleport(Pair(Matrix(0.0,0.0),pose))
        }
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
    l2.update()
    l1.draw()
    l2.draw()

    /*l1.rotate(PI/4)
    l2.update()
    l1.draw()
    l2.draw()

    l2.rotate(-PI/4)
    l1.draw()
    l2.draw()

    l1.rotate(PI/2)
    l2.update()
    l1.draw()
    l2.draw()*/

    l1.rotateTeleport(PI/2)
    l2.update()
    l1.draw()
    l2.draw()

    l2.rotateTeleport(-PI/4)
    l1.draw()
    l2.draw()

    l2.rotateTeleport(PI/4)
    l1.draw()
    l2.draw()

    l1.rotateTeleport(-PI/2)
    l2.update()
    l1.draw()
    l2.draw()

    l1.rotateTeleport(0.0)
    l2.update()
    l1.draw()
    l2.draw()

    l1.rotateTeleport(-PI)
    l2.update()
    l1.draw()
    l2.draw()
}