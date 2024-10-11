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
        cornerCoordinates.add(m * (f + Matrix(-length / 2, width / 2)))
        cornerCoordinates.add(m * (f + Matrix(length / 2, width / 2)))

        cornerCoordinates.add(m * (f + Matrix(length / 2, -width / 2)))
        cornerCoordinates.add(m * (f + Matrix(-length / 2, -width / 2)))
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

    fun collision(obj: Bot) {

    }
}