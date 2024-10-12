import edu.princeton.cs.introcs.StdDraw
import java.awt.Color

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
        val l:MutableList<(Int)->Boolean>
        //given endpoints a and b, and input point c, check if c satisfies the inequality formed by line a and b given if compare compares greater than or less than
        fun inequality(a:Matrix, b:Matrix, c:Matrix, compare:(Matrix)->Boolean) :Boolean {

            return false;
        }


        return false;
    }
}