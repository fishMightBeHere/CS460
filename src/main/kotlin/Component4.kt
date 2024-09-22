import edu.princeton.cs.introcs.StdDraw
import java.awt.Color

typealias Vector = Matrix


class Component4 {
    val l1 = Bot(
        frame = Pair(Matrix(0.0,0.0),0.0),
        points = listOf(Pair(5.0,-10.0), Pair(5.0,10.0), Pair(-5.0,10.0), Pair(-5.0,-10.0)),
        endEffector = Matrix(0.0,10.0,0.0),
        botColor = Color.pink,
    )
    val l2 = Bot(
        frame = Pair(Matrix(0.0,0.0),0.0),
        points = listOf(Pair(5.0,0.0), Pair(5.0,15.0), Pair(-5.0,15.0), Pair(-5.0,0.0)),
        endEffector = null,
        botColor = Color.ORANGE,
        root = l1
    )
    //bot length 1 is 2, bot2 is 1.5


    fun interpolate_arm(start: Vector, goal: Vector): Path {
        val step = 10
        val p:Path = Path(mutableListOf())
        return p
    }
//
//    fun forward_propagate_arm(start_pose: Vector, plan: Plan): Path {
//
//    }
//
//    fun visualize_arm_path(path: Path) {}

}

fun main() {
    val canvas = Drawer(1000,1000,0.5)
    canvas.axes()

    val c4 = Component4()

    c4.l1.draw()
    c4.l2.draw()


}
