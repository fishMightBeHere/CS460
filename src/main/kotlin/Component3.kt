import java.awt.Color
import kotlin.math.PI

data class Pose(val x: Double, val y: Double, val theta: Double) {
    constructor(m: Matrix) : this(m[0, 0], m[1, 0], m[2, 0])
}

data class Path<T>(val path: MutableList<T> = mutableListOf())

typealias Velocity = Pose

data class Plan<T>(val plan: Collection<Pair<T, Double>>)


class Component3 {
    companion object {
        val bot = Bot(
            Pair(Matrix(0.0, 0.0), 0.0),
            listOf(Pair(10.0, -20.0), Pair(10.0, 20.0), Pair(-10.0, 20.0), Pair(-10.0, -20.0)),
            null,
            Color.PINK
        )

        fun interpolate_rigid_body(start_pose: Matrix, goal_pose: Matrix): Path<Pose> {
            val dx = goal_pose[0, 0] - start_pose[0, 0]
            val dy = goal_pose[1, 0] - start_pose[1, 0]
            val dth = goal_pose[2, 0] - start_pose[2, 0]

            val path = Path<Pose>()

            val steps = 10
            for (i in 0..steps) {
                path.path.add(
                    Pose(
                        start_pose[0, 0] + (i * (dx / steps)),
                        start_pose[1, 0] + (i * (dy / steps)),
                        start_pose[2, 0] + (i * (dth / steps))
                    )
                )
            }

            return path
        }

        fun forward_propogate_rigid_body(start_pose: Matrix, plan: Plan<Velocity>): Path<Pose> {
            val path = Path<Pose>()
            var curPose = Pose(start_pose)
            path.path.add(curPose)
            for (a: Pair<Velocity, Double> in plan.plan) {
                curPose = Pose(a.first.x * a.second, a.first.y * a.second, a.first.theta * a.second)
                path.path.add(curPose)
            }
            return path
        }

        fun visualise_path(path: Path<Pose>) {
            for (pose in path.path) {
                //display list poses
                val dx = pose.x - bot.frame.first[0,0]
                val dy = pose.y - bot.frame.first[1,0]
                val dtheta = pose.theta - bot.frame.second

                val step = 0.01
                for (count in 0..<100) {
                    bot.move(Pair(Matrix(dx, dy) * step,step*dtheta))
                    bot.draw()
                    Thread.sleep(10)
                }

            }
        }
    }
}

fun main() {
    val canvas = Drawer(1000, 1000, 1.0)
    canvas.axes()
    Component3.visualise_path(Path(mutableListOf(Pose(100.0,100.0,PI))))



//     ground translations don't seem to quite work, it seems that somewhere a rotation value is lost
    /*ground.move(Pair(Matrix(50.0,50.0),0.0))
    arm1.update()
    arm2.update()
    arm1.draw()
    arm2.draw()*/

    // we need to implement a hierarchy of arms starting from ground towards last arm to allow for automatic arm updates
}