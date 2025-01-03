import edu.princeton.cs.introcs.StdDraw
import java.awt.Color
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

// frame: Matrix [x,y] | theta relative to origin
// endEffector : Matrix [x,y], theta relative to origin, like a point but with a rotation, theta of end effector should be the same as frame to prevent confusion
class Bot(
    var frame: Pair<Matrix, Double>,
    points: List<Pair<Double, Double>>,
    endEffector: Pair<Matrix, Double>? = null,
    var botColor: Color,
    val root: Bot? = null,
    var displayFrame: Boolean = true
) {
    val cornerVector: MutableList<Matrix> = mutableListOf()
    var vectorToEndEffector: Pair<Matrix, Double>? = null

    init {
        // set vectors from frame towards the 4 corners
        for ((x, y) in points) {
            cornerVector.add(Matrix(x - frame.first[0, 0], y - frame.first[1, 0]))
        }
        if (endEffector != null) {
            vectorToEndEffector = Pair(
                Matrix(
                    endEffector.first[0, 0] - frame.first[0, 0],
                    endEffector.first[1, 0] - frame.first[1, 0]
                ), endEffector.second
            )
        }
        if (root != null) frame = Pair(
            LinearAlgebra.rotationMatrixR2(root.frame.second) * root.vectorToEndEffector!!.first + root.frame.first,
            root.frame.second
        )
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
        if (xValues.isNotEmpty()) {
            StdDraw.setPenColor(botColor)
            StdDraw.filledPolygon(xValues.toDoubleArray(), yValues.toDoubleArray())
            StdDraw.setPenColor(Color.GREEN)
        }

        if (displayFrame) {// draw frame
            StdDraw.circle(frameXY[0, 0], frameXY[1, 0], 5.0)
            //draw positive x axis
            StdDraw.setPenColor(Color.RED)
            StdDraw.line(
                frameXY[0, 0],
                frameXY[1, 0],
                (rot * Matrix(10.0, 0.0) + frameXY)[0, 0],
                (rot * Matrix(10.0, 0.0) + frameXY)[1, 0]
            )
            //draw positive y axis
            StdDraw.setPenColor(Color.GREEN)
            StdDraw.line(
                frameXY[0, 0],
                frameXY[1, 0],
                (rot * Matrix(0.0, 10.0) + frameXY)[0, 0],
                (rot * Matrix(0.0, 10.0) + frameXY)[1, 0]
            )
        }

        if (vectorToEndEffector != null) {//draw endEffectors
            val efm = LinearAlgebra.rotationMatrixR2(frame.second) * vectorToEndEffector!!.first + frame.first
            val t = frame.second
            StdDraw.setPenColor(Color.BLUE)
            StdDraw.circle(efm[0, 0], efm[1, 0], 5.0)
            StdDraw.setPenColor(Color.RED)
            val efRot = Matrix(doubleArrayOf(cos(t), -sin(t)), doubleArrayOf(sin(t), cos(t)))
            //draw positive x axis
            StdDraw.setPenColor(Color.RED)
            StdDraw.line(
                efm[0, 0],
                efm[1, 0],
                (efRot * Matrix(5.0, 0.0) + efm)[0, 0],
                (efRot * Matrix(5.0, 0.0) + efm)[1, 0]
            )
            //draw positive y axis
            StdDraw.setPenColor(Color.GREEN)
            StdDraw.line(
                efm[0, 0],
                efm[1, 0],
                (efRot * Matrix(0.0, 5.0) + efm)[0, 0],
                (efRot * Matrix(0.0, 5.0) + efm)[1, 0]
            )
        }

    }

    //v:Matrix -> [x translation, y translation, theta rotation]
    fun move(v: Pair<Matrix, Double>) {
        var (frameXY, theta) = frame
        if (root != null) {
            root.vectorToEndEffector =
                Pair(root.vectorToEndEffector!!.first, root.vectorToEndEffector!!.second + v.second)

            frameXY =
                LinearAlgebra.rotationMatrixR2(root.frame.second) * root.vectorToEndEffector!!.first + root.frame.first
            theta = root.vectorToEndEffector!!.second
            root.vectorToEndEffector = Pair(root.vectorToEndEffector!!.first, theta)
        } else {
            frameXY += v.first
            theta += v.second
        }
        frame = Pair(frameXY, theta)


        if (vectorToEndEffector != null) {//rotates vectors pointing to end effectors
            val (efXY, thetaEF) = vectorToEndEffector!!
            vectorToEndEffector = Pair(efXY, thetaEF + v.second)
        }
    }

    fun teleport(f: Pair<Matrix, Double>) {
        if (vectorToEndEffector != null) {
            val localPose =  vectorToEndEffector!!.second - frame.second
            vectorToEndEffector = Pair(vectorToEndEffector!!.first,(f.second+ localPose)%(2*PI))
        }
        if (root != null) {
            frame = Pair(
                LinearAlgebra.rotationMatrixR2(root.frame.second) * root.vectorToEndEffector!!.first + root.frame.first,
                root.frame.second + f.second
            )
            root.vectorToEndEffector = Pair(root.vectorToEndEffector!!.first,root.frame.second + f.second)

        } else frame = f

    }

    fun update() {
        move(Pair(Matrix(0.0, 0.0), 0.0))
    }

    fun rotate(theta: Double) {
        move(Pair(Matrix(0.0, 0.0), theta))
    }

    fun rotateTeleport(theta: Double) {
        teleport(Pair(Matrix(0.0, 0.0), theta))
    }
}