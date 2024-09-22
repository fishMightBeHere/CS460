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
    private val endEffector: Matrix?,
    private val botColor: Color,
    val root:Bot? = null,
) {
    private val cornerVector: MutableList<Matrix> = mutableListOf()
    var vectorToEndEffector: Pair<Matrix, Double>? = null
    var transformedEF: Pair<Matrix,Double>? = null

    init {
        // set vectors from frame towards the 4 corners
        for ((x, y) in points) {
            cornerVector.add(Matrix(x - frame.first[0, 0], y - frame.first[1, 0]))
        }
        if (endEffector != null) {
            vectorToEndEffector = Pair(Matrix(endEffector[0, 0] - frame.first[0, 0], endEffector[1, 0] - frame.first[1, 0]), endEffector[2, 0])
            transformedEF = Pair(Matrix(endEffector[0, 0] - frame.first[0, 0], endEffector[1, 0] - frame.first[1, 0]), endEffector[2, 0])
        }
        if (root != null) frame = root.getEndEffector()!!
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
        if (!xValues.isEmpty()) {
            StdDraw.setPenColor(botColor)
            StdDraw.filledPolygon(xValues.toDoubleArray(), yValues.toDoubleArray())
            StdDraw.setPenColor(Color.GREEN)
        }

        // draw frame
        StdDraw.circle(frameXY[0, 0], frameXY[1, 0], 5.0)
        //draw positive x axis
        StdDraw.setPenColor(Color.RED)
        StdDraw.line(frameXY[0,0],frameXY[1,0], (rot*Matrix(10.0,0.0)+frameXY)[0,0], (rot*Matrix(10.0,0.0)+frameXY)[1,0])
        //draw positive y axis
        StdDraw.setPenColor(Color.GREEN)
        StdDraw.line(frameXY[0,0],frameXY[1,0], (rot*Matrix(0.0,10.0)+frameXY)[0,0], (rot*Matrix(0.0,10.0)+frameXY)[1,0])

        if (transformedEF != null) {//draw endEffectors
            val (efm, t) = transformedEF!!
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
        if (root != null) {
            root.vectorToEndEffector = Pair(root.vectorToEndEffector!!.first, root.vectorToEndEffector!!.second+v.second)

            frameXY = root.transformedEF!!.first
            theta = root.vectorToEndEffector!!.second


            //root.vectorToEndEffectors = Pair(root.vectorToEndEffectors!!.first,root.vectorToEndEffectors!!.second + theta)

        } else {
            frameXY += v.first
            theta += v.second
        }
        frame = Pair(frameXY, theta)

        val rot = Matrix(2, 2)
        rot[0, 0] = cos(v.second)
        rot[0, 1] = -sin(v.second)
        rot[1, 0] = sin(v.second)
        rot[1, 1] = cos(v.second)

        if (vectorToEndEffector != null) {//rotates vectors pointing to end effectors
            val (efXY, thetaEF) = vectorToEndEffector!!
            val frameRot = Matrix(doubleArrayOf(cos(theta),-sin(theta)), doubleArrayOf(sin(theta), cos(theta)))
            vectorToEndEffector = Pair(efXY,thetaEF+v.second)
            transformedEF= Pair(frameRot*efXY+frameXY,thetaEF+v.second)
        }

    }

    fun update() {
        move(Pair(Matrix(0.0,0.0),0.0))
    }

    fun rotate(theta:Double) {
        move(Pair(Matrix(0.0,0.0),theta))
    }

    fun getEndEffector(): Pair<Matrix, Double>? {
        return transformedEF
    }
}


class Component3 {

    companion object {
        val bot = Bot(
            Pair(Matrix(0.0, 0.0), 0.0),
            listOf(Pair(10.0, -20.0), Pair(10.0, 20.0), Pair(-10.0, 20.0), Pair(-10.0, -20.0)),
            null,
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
    val canvas = Drawer(1000, 1000, 0.5)
    canvas.axes()
    val ground = Bot(
        Pair(Matrix(0.0,0.0),0.0),
        listOf(),
        Matrix(0.0,0.0,0.0),
        Color.black
    )
    val arm1 = Bot(
        Pair(Matrix(0.0, 0.0), 0.0),
        listOf(Pair(10.0, 0.0), Pair(10.0, 40.0), Pair(-10.0, 40.0), Pair(-10.0, 0.0)),
        Matrix(0.0,40.0,0.0),
        botColor = Color.PINK,
        root = ground
    )
    val arm2 = Bot(
        Pair(Matrix(0.0,0.0),0.0),
        listOf(Pair(10.0,0.0),Pair(10.0,20.0),Pair(-10.0,20.0),Pair(-10.0,0.0)),
        null,
        Color.ORANGE,
        arm1
    )

    arm2.update()
    arm1.draw()
    arm2.draw()

    arm1.rotate(PI/2)
    arm2.update()
    arm1.draw()
    arm2.draw()

    arm2.rotate(PI/2)
    arm1.draw()
    arm2.draw()

    arm1.rotate(PI/2)
    arm2.update()
    arm1.draw()
    arm2.draw()

    arm1.rotate(PI/2)
    arm2.update()
    arm1.draw()
    arm2.draw()


//     ground translations don't seem to quite work, it seems that somewhere a rotation value is lost
    /*ground.move(Pair(Matrix(50.0,50.0),0.0))
    arm1.update()
    arm2.update()
    arm1.draw()
    arm2.draw()*/

    // we need to implement a hierarchy of arms starting from ground towards last arm to allow for automatic arm updates

}