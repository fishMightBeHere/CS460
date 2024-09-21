import edu.princeton.cs.introcs.StdDraw
import kotlin.math.cos
import kotlin.math.sin

data class Pose(val x:Double,val y:Double, val theta:Double ) {
    constructor(m:Matrix) : this(m[0,0],m[1,0],m[2,0])
}

data class Path(val path:MutableList<Pose> = mutableListOf())

typealias Velocity = Pose

data class Plan(val plan: Collection<Pair<Velocity,Double>>)

//input coordinates of the 4 corners of the bot and the coordinates for the frame
class Bot(val corner1:Pair<Double,Double>, val corner2:Pair<Double,Double>, val corner3:Pair<Double,Double>, val corner4:Pair<Double,Double>, val frameX:Double, val frameY:Double) {
    private val cornerVector:MutableList<Matrix> = mutableListOf()
    private val frameXY = Matrix(doubleArrayOf(frameX,frameY))
    init {
        // set vectors from frame towards the 4 corners
        cornerVector.add(Matrix(doubleArrayOf(corner1.first-frameX,corner1.second-frameY)))
        cornerVector.add(Matrix(doubleArrayOf(corner2.first-frameX,corner2.second-frameY)))
        cornerVector.add(Matrix(doubleArrayOf(corner3.first-frameX,corner3.second-frameY)))
        cornerVector.add(Matrix(doubleArrayOf(corner4.first-frameX,corner4.second-frameY)))
    }
    fun draw(pose: Pose) {
        // convert pose into rotation and translation matrices and vectors
        val rot = Matrix(2,2)
        rot[0,0] = cos(pose.theta)
        rot[0,1] = -sin(pose.theta)
        rot[1,0] = sin(pose.theta)
        rot[1,1] = cos(pose.theta)

        // point translation = cv * rot +pose
        val transformedMatrices = mutableListOf<Matrix>()
        for (cv in cornerVector) {
            transformedMatrices.add((rot*cv)+frameXY+Matrix(doubleArrayOf(pose.x,pose.y)))
        }

        val xValues = mutableListOf<Double>()
        val yValues = mutableListOf<Double>()
        for (m in transformedMatrices) {
            xValues.add(m[0,0])
            yValues.add(m[1,0])
        }
        StdDraw.polygon(xValues.toDoubleArray(),yValues.toDoubleArray())
    }

}

class Component3 {
    companion object {
        fun interpolate_rigid_body(start_pose:Matrix, goal_pose:Matrix) : Path {
            val dx = goal_pose[0,0] - start_pose[0,0]
            val dy = goal_pose[1,0] - start_pose[1,0]
            val dth = goal_pose[2,0] - start_pose[2,0]

            val path = Path()

            val steps = 10
            for (i in 0..steps) {
                path.path.add(Pose(start_pose[0,0]+(i*(dx/steps)), start_pose[1,0]+(i*(dy/steps)), start_pose[2,0]+(i*(dth/steps))))
            }

            return path
        }

        fun forward_propogate_rigid_body(start_pose: Matrix, plan:Plan ) : Path {
            val path = Path()
            var curPose = Pose(start_pose)
            path.path.add(curPose)
            for(a:Pair<Velocity,Double> in plan.plan) {
                curPose = Pose(a.first.x*a.second, a.first.y*a.second, a.first.theta*a.second)
                path.path.add(curPose)
            }
            return path
        }

        fun visualise_path(path:Path) {
            for (pose in path.path) {
                //display list poses
            }
        }
    }
}

fun main() {
    val canvas = Drawer(1000,1000,1)
    canvas.axes()
    val bot = Bot(Pair(10.0,-20.0),Pair(10.0,20.0),Pair(-10.0,20.0),Pair(-10.0,-20.0),0.0,0.0)
    bot.draw(Pose(100.0,-100.0,Math.PI/4))
    print("done")
}