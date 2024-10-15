import edu.princeton.cs.introcs.StdDraw
import java.awt.Color
import kotlin.math.PI

class SquareObstacle(
    val frame: Pair<Matrix, Double>, val length: Double, val width: Double, val color: Color = Color.BLACK
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
            xVal.add(p[0, 0])
            yVal.add(p[1, 0])
        }

        if (xVal.isNotEmpty()) {
            StdDraw.setPenColor(color)
            StdDraw.filledPolygon(xVal.toDoubleArray(), yVal.toDoubleArray())
        }
    }

    //returns true if there is a collision, false if there is no collision
    fun isCollision(obj: Bot): Boolean {
        val obsf: MutableList<(Matrix) -> Boolean> = mutableListOf()

        for (i in 0..<cornerCoordinates.size) {
            obsf.add { m: Matrix ->
                val dxdy = cornerCoordinates[(i + 1) % cornerCoordinates.size] - cornerCoordinates[i]
                if (dxdy[0, 0] == 0.0) !(frame.first[0, 0] <= cornerCoordinates[i][0, 0]).xor(m[0, 0] <= cornerCoordinates[i][0, 0])
                else if (dxdy[1, 0] == 0.0) !(frame.first[1, 0] <= cornerCoordinates[i][1, 0]).xor(m[1, 0] <= cornerCoordinates[i][1, 0])
                else !(frame.first[1, 0] - cornerCoordinates[i][1, 0] <= (dxdy[1, 0] / dxdy[0, 0]) * (frame.first[0, 0] - cornerCoordinates[i][0, 0])).xor(
                    m[1, 0] - cornerCoordinates[i][1, 0] <= (dxdy[1, 0] / dxdy[0, 0]) * (m[0, 0] - cornerCoordinates[i][0, 0])
                )
            }
        }
        // caluclate xy coordinates of bot
        val corners = obj.cornerVector.map { LinearAlgebra.rotationMatrixR2(obj.frame.second) * it + obj.frame.first }
            .toMutableList()
        corners.add(obj.frame.first)

        for (corner in corners) {
            if (!obsf.map { it(corner) }.contains(false)) return true
        }

        corners.removeLast()
        val botsf: MutableList<(Matrix) -> Boolean> = mutableListOf()

        for (i in 0..<corners.size) {
            botsf.add { m: Matrix ->
                val dxdy = corners[(i + 1) % corners.size] - corners[i]
                if (dxdy[0, 0] == 0.0) !(obj.frame.first[0, 0] <= corners[i][0, 0]).xor(m[0, 0] <= corners[i][0, 0])
                else if (dxdy[1, 0] == 0.0) !(obj.frame.first[1, 0] <= corners[i][1, 0]).xor(m[1, 0] <= corners[i][1, 0])
                else !(obj.frame.first[1, 0] - corners[i][1, 0] <= (dxdy[1, 0] / dxdy[0, 0]) * (obj.frame.first[0, 0] - corners[i][0, 0])).xor(
                    m[1, 0] - corners[i][1, 0] <= (dxdy[1, 0] / dxdy[0, 0]) * (m[0, 0] - corners[i][0, 0])
                )
            }
        }

        cornerCoordinates.add(frame.first)
        for (corner in cornerCoordinates) {
            if (!botsf.map {it(corner)}.contains(false)) return true
        }
        cornerCoordinates.removeLast()

        return false;// don't collide
    }
}

fun main() {
    val obs = SquareObstacle(Pair(Matrix(0.0, 0.0), 0.0), 20.0, 20.0)
    val bot = Bot(
        Pair(Matrix(0.0, 0.0), 0.0),
        mutableListOf(Pair(10.0, 10.0), Pair(10.0, -10.0), Pair(-10.0, -10.0), Pair(-10.0, 10.0)),
        botColor = Color.BLUE
    )
    Drawer(500, 500, 0.2)
    bot.teleport(Pair(Matrix(20.0, 15.0), PI / 4))
    bot.draw()
    obs.draw()
    if (obs.isCollision(bot)) println("they collide") else println("they don't collide")
}