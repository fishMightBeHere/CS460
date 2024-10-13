import edu.princeton.cs.introcs.StdDraw
import java.awt.Color
import kotlin.time.times

class SquareObstacle(
    val frame: Pair<Matrix, Double>,
    val length: Double,
    val width: Double,
    private val color: Color = Color.BLACK
) {
    val cornerCoordinates: MutableList<Matrix> = mutableListOf()

    init {
        // generate cornerCoordinates
        val (f, t) = frame;
        val m = LinearAlgebra.rotationMatrixR2(t)
        cornerCoordinates.add(m * (f + Matrix(-length / 2, width / 2)))//top left
        cornerCoordinates.add(m * (f + Matrix(length / 2, width / 2)))//top right
        cornerCoordinates.add(m * (f + Matrix(length / 2, -width / 2)))//bottom right
        cornerCoordinates.add(m * (f + Matrix(-length / 2, -width / 2)))//bottom left
    }

    fun draw() {
        // break cornerCoordinates into list of x and y coordinates
        val xVal = mutableListOf<Double>()
        val yVal = mutableListOf<Double>()
        for (p in cornerCoordinates) {
            xVal.add(p[0,0])
            yVal.add(p[1,0])
        }

        if (xVal.isNotEmpty()) {
            StdDraw.setPenColor(color)
            StdDraw.filledPolygon(xVal.toDoubleArray(), yVal.toDoubleArray())
        }
    }

    fun isCollision(obj: Bot) : Boolean{
        // construct parametric lines to represent edges of obstacle and bot, take each corner of opposite object and check if it satisfies all inequalities
        // x(t) = x0 + tdx
        // y(t) = y0 + tdy
        val l:MutableList<(Matrix)->Boolean> = mutableListOf()

        for (i in 0..<cornerCoordinates.size) {
            l.add { m: Matrix ->
                val dxdy = cornerCoordinates[(i+1)%cornerCoordinates.size] - cornerCoordinates[i]
                val t = if (dxdy[0,0] == 0.0) 0.0 else (m[0,0] - cornerCoordinates[i][0,0])/dxdy[0,0]
                // check if input point and frame are on same side of line
                return@add !(m[1,0] <= (cornerCoordinates[i][1,0] + dxdy[1,0]*t)) xor (frame.first[1,0] <= cornerCoordinates[i][1,0] + dxdy[1,0]*t)
            }
        }
        // caluclate xy coordinates of bot
        val corners = obj.cornerVector.map { LinearAlgebra.rotationMatrixR2(obj.frame.second)*it + obj.frame.first}.toList()
        l.zip(corners).map { (f,p) ->  f(p) }.filter { it }.ifEmpty { return true } //note this only compares obj's corners to the obstacle, we also need to check center and compare corners with each other
        return false;
    }
}