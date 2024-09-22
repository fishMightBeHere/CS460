import edu.princeton.cs.introcs.StdDraw
import java.awt.Color

typealias Vector = Matrix

/*
class Component4 {
    val ground = Bot(Pair(0.0,0.0),Pair(0.0,0.0),Pair(0.0,0.0),Pair(0.0,0.0),0.0,0.0,Matrix(doubleArrayOf(0.0,0.0)))
    val l1 = Bot(Pair(5.0, 0.0), Pair(5.0, 20.0), Pair(-5.0, 20.0), Pair(-5.0, 0.0), 0.0, 0.0, endEffector = Matrix(
        doubleArrayOf(0.0,20.0)))
    val l2 = Bot(Pair(5.0, 0.0), Pair(5.0, 15.0), Pair(-5.0, 15.0), Pair(-5.0, 0.0), 0.0, 0.0)
    //bot length 1 is 2, bot2 is 1.5


//    fun interpolate_arm(start: Vector, goal: Vector): Path {
//
//    }
//
//    fun forward_propagate_arm(start_pose: Vector, plan: Plan): Path {
//
//    }
//
//    fun visualize_arm_path(path: Path) {}

}

fun main() {
    val canvas = Drawer(1000,1000,1.0)
    canvas.axes()

    val c4 = Component4()
    c4.l1.draw(Pose(Matrix(doubleArrayOf(c4.ground.endEffector?.get(0, 0) ?: 0.0, c4.ground.endEffector?.get(1, 0) ?: 0.0,Math.PI/4))))
    c4.l2.curFrame = c4.l1.endEffector!!
    StdDraw.setPenColor(Color.RED)
    c4.l2.draw(Pose(Matrix(doubleArrayOf(c4.l1.curEndEffector?.get(0, 0) ?: 0.0, c4.l1.curEndEffector?.get(1, 0) ?: 0.0, 0.0))))
}*/
