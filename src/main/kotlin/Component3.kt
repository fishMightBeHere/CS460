import edu.princeton.cs.introcs.StdDraw
import java.awt.Color
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

data class Pose(val x: Double, val y: Double, val theta: Double) {
    constructor(m: Matrix) : this(m[0, 0], m[1, 0], m[2, 0])
}

data class Path(val path: MutableList<Pose> = mutableListOf())

typealias Velocity = Pose

data class Plan(val plan: Collection<Pair<Velocity, Double>>)

// frame: Matrix [x,y] | theta relative to origin
// endEffector : Matrix [x,y,theta] relative to origin, like a point but with a rotation, theta of end effector should be the same as frame to prevent confusion
class Bot(
    var frame: Pair<Matrix, Double>,
    private val points: List<Pair<Double, Double>>,
    private val endEffectors: List<Matrix>,
    private val botColor: Color
) {
    private val cornerVector: MutableList<Matrix> = mutableListOf()
    val vectorsToEndEffectors: MutableList<Pair<Matrix, Double>> = mutableListOf()
    val transformedEF: MutableList<Pair<Matrix,Double>> = mutableListOf()

    // [vx,vy], theta stores vector from frame pointing to each end effector, also stores that end effector's cur rotation

    init {
        // set vectors from frame towards the 4 corners
        for ((x, y) in points) {
            cornerVector.add(Matrix(x - frame.first[0, 0], y - frame.first[1, 0]))
        }

        for (ef in endEffectors) {
            vectorsToEndEffectors.add(Pair(Matrix(ef[0, 0] - frame.first[0, 0], ef[1, 0] - frame.first[1, 0]), ef[2, 0]))
            transformedEF.add(Pair(Matrix(ef[0, 0] - frame.first[0, 0], ef[1, 0] - frame.first[1, 0]), ef[2, 0]))
        }
    }

    fun draw() {
        val (frameXY, theta) = frame

        val rot = Matrix(2, 2)
        rot[0, 0] = cos(theta)
        rot[0, 1] = -sin(theta)
        rot[1, 0] = sin(theta)
        rot[1, 1] = cos(theta)

        val transformedMatrices = mutableListOf<Matrix>()
        for (cv in cornerVector) {
            transformedMatrices.add((rot * cv) + frameXY)
        }

        val xValues = mutableListOf<Double>()
        val yValues = mutableListOf<Double>()
        for (m in transformedMatrices) {
            xValues.add(m[0, 0])
            yValues.add(m[1, 0])
        }
        StdDraw.setPenColor(botColor)
        StdDraw.filledPolygon(xValues.toDoubleArray(), yValues.toDoubleArray())
        StdDraw.setPenColor(Color.GREEN)

        // draw frame
        StdDraw.circle(frameXY[0, 0], frameXY[1, 0], 5.0)
        //draw positive x axis
        StdDraw.setPenColor(Color.RED)
        StdDraw.line(frameXY[0,0],frameXY[1,0], (rot*Matrix(10.0,0.0)+frameXY)[0,0], (rot*Matrix(10.0,0.0)+frameXY)[1,0])
        //draw positive y axis
        StdDraw.setPenColor(Color.GREEN)
        StdDraw.line(frameXY[0,0],frameXY[1,0], (rot*Matrix(0.0,10.0)+frameXY)[0,0], (rot*Matrix(0.0,10.0)+frameXY)[1,0])

        //draw endEffectors
        for (ef in transformedEF) {
            val (efm, t) = ef
            StdDraw.setPenColor(Color.BLUE)
            StdDraw.circle(efm[0,0],efm[1,0],5.0)
            StdDraw.setPenColor(Color.RED)
            val efRot = Matrix(doubleArrayOf(cos(t),-sin(t)), doubleArrayOf(sin(t), cos(t)))
            //draw positive x axis
            StdDraw.setPenColor(Color.RED)
            StdDraw.line(efm[0,0],efm[1,0], (efRot*Matrix(5.0,0.0)+efm)[0,0], (efRot*Matrix(5.0,0.0)+efm)[1,0])
            //draw positive y axis
            StdDraw.setPenColor(Color.GREEN)
            StdDraw.line(efm[0,0],efm[1,0], (efRot*Matrix(0.0,5.0)+efm)[0,0], (efRot*Matrix(0.0,5.0)+efm)[1,0])
        }
    }

    //v:Matrix -> [x translation, y translation, theta rotation]
    fun move(v: Pair<Matrix, Double>) {
        var (frameXY, theta) = frame
        frameXY += v.first
        theta += v.second

        frame = Pair(frameXY, theta)

        val rot = Matrix(2, 2)
        rot[0, 0] = cos(v.second)
        rot[0, 1] = -sin(v.second)
        rot[1, 0] = sin(v.second)
        rot[1, 1] = cos(v.second)

        //rotates vectors pointing to end effectors
        for ((i, ef) in vectorsToEndEffectors.withIndex()) {
            val (efXY, thetaEF) = ef
            val frameRot = Matrix(doubleArrayOf(cos(theta),-sin(theta)), doubleArrayOf(sin(theta), cos(theta)))
            transformedEF[i] = (Pair(frameRot*efXY+frameXY,thetaEF+theta))
        }
    }

    fun getEndEffectors(): List<Pair<Matrix, Double>> {
        val l = mutableListOf<Pair<Matrix, Double>>()
        for ((ef, theta) in vectorsToEndEffectors) {
            // add frame + vector to point, angle relative to origin axes is also summed
            l.add(Pair(Matrix(ef[0, 0] + frame.first[0, 0]), theta + frame.second))
        }
        return l
    }
}


class Component3 {

    companion object {
        val bot = Bot(
            Pair(Matrix(0.0, 0.0), 0.0),
            listOf(Pair(10.0, -20.0), Pair(10.0, 20.0), Pair(-10.0, 20.0), Pair(-10.0, -20.0)),
            listOf(),
            Color.PINK
        )

        fun interpolate_rigid_body(start_pose: Matrix, goal_pose: Matrix): Path {
            val dx = goal_pose[0, 0] - start_pose[0, 0]
            val dy = goal_pose[1, 0] - start_pose[1, 0]
            val dth = goal_pose[2, 0] - start_pose[2, 0]

            val path = Path()

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

        fun forward_propogate_rigid_body(start_pose: Matrix, plan: Plan): Path {
            val path = Path()
            var curPose = Pose(start_pose)
            path.path.add(curPose)
            for (a: Pair<Velocity, Double> in plan.plan) {
                curPose = Pose(a.first.x * a.second, a.first.y * a.second, a.first.theta * a.second)
                path.path.add(curPose)
            }
            return path
        }

        fun visualise_path(path: Path) {
            for (pose in path.path) {
                //display list poses
                val dx = pose.x - bot.frame.first[0,0]
                val dy = pose.y - bot.frame.first[1,0]
                val dtheta = pose.theta - bot.frame.second
                bot.move(Pair(Matrix(dx,dy),dtheta))
            }
        }
    }
}

fun main() {
    val canvas = Drawer(1000, 1000, 1.0)
    canvas.axes()
    val bot = Bot(
        Pair(Matrix(0.0, 0.0), 0.0),
        listOf(Pair(10.0, 0.0), Pair(10.0, 20.0), Pair(-10.0, 20.0), Pair(-10.0, 0.0)),
        listOf(Matrix(0.0,20.0,0.0)),
        Color.PINK
    )


    bot.move(Pair(Matrix(100.0,100.0), PI/4))
    bot.draw()
    bot.move(Pair(Matrix(-100.0,0.0),-PI/4))
    bot.draw()
    bot.move(Pair(Matrix(-45.0,-200.0),-3*PI/4))
    bot.draw()
}