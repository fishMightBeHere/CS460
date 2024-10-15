import edu.princeton.cs.introcs.StdDraw
import java.awt.Color
import kotlin.math.PI

typealias Vector = Matrix

data class ArmPath(val path: MutableList<Pair<Pose, Pose>>)

typealias AnglePose = Pair<Double, Double>

class A1C4 {
    val l1 = Bot(
        frame = Pair(Matrix(0.0, 0.0), 0.0),
        points = listOf(Pair(5.0, -10.0), Pair(5.0, 10.0), Pair(-5.0, 10.0), Pair(-5.0, -10.0)),
        endEffector = Matrix(0.0, 10.0, 0.0),
        botColor = Color.pink,
    )
    val l2 = Bot(
        frame = Pair(Matrix(0.0, 0.0), 0.0),
        points = listOf(Pair(5.0, 0.0), Pair(5.0, 15.0), Pair(-5.0, 15.0), Pair(-5.0, 0.0)),
        endEffector = null,
        botColor = Color.ORANGE,
        root = l1
    )
    //bot length 1 is 2, bot2 is 1.5

    //[t1, t2], [t1,t2]
    fun interpolate_arm(start: Vector, goal: Vector): ArmPath {
        val step = 0.1
        val p = ArmPath(mutableListOf())
        val dtheta1 = goal[0, 0] - start[0, 0]
        val dtheta2 = goal[1, 0] - start[1, 0]

        var dt = 0.0
        while (dt <= 1) {
            p.path.add(
                Pair(
                    Pose(0.0, 0.0, start[0, 0] + dtheta1 * dt),
                    Pose(0.0, 0.0, start[1, 0] + dtheta2 * dt)
                )
            )
            dt += step
        }
        return p
    }

    //plan stores angular velocities of joints w1, w2, returns a path with pose represented as a pair of angles
    fun forward_propagate_arm(start_pose: Vector, plan: Plan<AnglePose>): Path<AnglePose> {
        val p: MutableList<AnglePose> = mutableListOf()
        for ((v, t) in plan.plan) {
            p.add(Pair(start_pose[0, 0] + v.first * t, start_pose[1, 0] + v.second * t))
        }
        return Path(p)
    }

    fun visualize_arm_path(path: Path<AnglePose>) {
        for ((theta1, theta2) in path.path) {
            val dtheta1 = theta1 - l1.frame.second
            val dtheta2 = theta2 + l1.frame.second - l2.frame.second

            val step = 0.01
            for (i in 0..<100) {
                StdDraw.clear()
                l1.rotate(step * dtheta1)
                l2.update()
                l2.rotate(step * dtheta2)
                l1.draw()
                l2.draw()
                Thread.sleep(1)
            }


        }
    }

}

fun main() {
    val canvas = Drawer(1000, 1000, drawScale = 0.25)
    canvas.axes()

    val c4 = A1C4()

    c4.visualize_arm_path(
        Path(
            mutableListOf(
                Pair(PI/4,0.0),
                Pair(-PI / 4,-PI/4),
                Pair(0.0,0.0),
                Pair(PI,0.0),
                Pair(0.0,PI),
                Pair(0.0,0.0)
            )
        )
    )
    print("done")

}
