import edu.princeton.cs.introcs.StdDraw
import java.awt.Color
import kotlin.math.PI

class SquareObstacle(
    val frame: Pair<Matrix, Double>, val length: Double, val width: Double, var color: Color = Color.GREEN
) {
    val cornerCoordinates: MutableList<Matrix> = mutableListOf()

    init {
        // generate cornerCoordinates
        val (f, t) = frame;
        val m = LinearAlgebra.rotationMatrixR2(t)
        cornerCoordinates.add((m * Matrix(-length / 2, width / 2)) + f)//top left
        cornerCoordinates.add((m * Matrix(length / 2, width / 2)) + f)//top right
        cornerCoordinates.add((m * Matrix(length / 2, -width / 2)) + f)//bottom right
        cornerCoordinates.add((m * Matrix(-length / 2, -width / 2))+f)//bottom left
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
    // requires all frames to be inside the robot because we need a point of reference, if there is a frame on the edge, than it does not tell which side is actually on the inside
    fun isCollision(obj: Bot): Boolean {
        val obsf: MutableList<(Matrix) -> Boolean> = mutableListOf()

        for (i in 0..<cornerCoordinates.size) {
            obsf.add { m: Matrix ->
                val dxdy = cornerCoordinates[(i + 1) % cornerCoordinates.size] - cornerCoordinates[i]
                if (dxdy[0, 0] == 0.0) !(frame.first[0, 0] < cornerCoordinates[i][0, 0]).xor(m[0, 0] < cornerCoordinates[i][0, 0])
                else if (dxdy[1, 0] == 0.0) !(frame.first[1, 0] < cornerCoordinates[i][1, 0]).xor(m[1, 0] < cornerCoordinates[i][1, 0])
                else !(frame.first[1, 0] - cornerCoordinates[i][1, 0] < (dxdy[1, 0] / dxdy[0, 0]) * (frame.first[0, 0] - cornerCoordinates[i][0, 0])).xor(
                    m[1, 0] - cornerCoordinates[i][1, 0] < (dxdy[1, 0] / dxdy[0, 0]) * (m[0, 0] - cornerCoordinates[i][0, 0])
                )
            }
        }
        // caluclate xy coordinates of bot
        val botCorners =
            obj.cornerVector.map { LinearAlgebra.rotationMatrixR2(obj.frame.second) * it + obj.frame.first }
                .toMutableList()
        botCorners.add(obj.frame.first)

        for (corner in botCorners) {
            if (obsf.map { it(corner) }.reduce { acc, b -> acc && b }) {
                //println("collision detected with point $corner from bot")
                return true
            }
        }
        botCorners.removeLast()
        val botsf: MutableList<(Matrix) -> Boolean> = mutableListOf()

        for (i in 0..<botCorners.size) {
            botsf.add { m: Matrix ->
                val dxdy = botCorners[(i + 1) % botCorners.size] - botCorners[i]
                if (dxdy[0, 0] == 0.0) !(obj.frame.first[0, 0] < botCorners[i][0, 0]).xor(m[0, 0] < botCorners[i][0, 0])
                else if (dxdy[1, 0] == 0.0) !(obj.frame.first[1, 0] < botCorners[i][1, 0]).xor(m[1, 0] < botCorners[i][1, 0])
                else !(obj.frame.first[1, 0] - botCorners[i][1, 0] < (dxdy[1, 0] / dxdy[0, 0]) * (obj.frame.first[0, 0] - botCorners[i][0, 0])).xor(
                    m[1, 0] - botCorners[i][1, 0] < (dxdy[1, 0] / dxdy[0, 0]) * (m[0, 0] - botCorners[i][0, 0])
                )
            }
        }

        cornerCoordinates.add(frame.first)
        for (corner in cornerCoordinates) {
            if (botsf.map { it(corner) }.reduce { acc, b -> acc && b }) {
                //println("collision detected with point $corner from obstacle")
                cornerCoordinates.removeLast()
                return true
            }
        }
        cornerCoordinates.removeLast()
        return false;// don't collide
    }
}

fun main() {
    //val obs = SquareObstacle(Pair(Matrix(-27.4525911, 12.704756426), 4.45809), 11.324, 16.83)
    val obs = SquareObstacle(Pair(Matrix(0.0,0.0), 4.45809), 11.324, 16.83)
    val bot = Bot(
        Pair(Matrix(0.0, 0.0), 0.0),
        mutableListOf(Pair(10.0, 20.0), Pair(10.0, -1.0), Pair(-10.0, -1.0), Pair(-10.0, 20.0)),
        botColor = Color.BLUE
    )
    Drawer(500, 500, drawScale = 0.3)
    bot.teleport(Pair(Matrix(25.0, 35.0), 10.924856148387873))
    bot.draw()
    //obs.isCollision(bot)

    var r = 0.0
    while (!StdDraw.mousePressed()) {
        r= (r+0.01)%(2*PI)
        bot.teleport(Pair(Matrix(StdDraw.mouseX(), StdDraw.mouseY()), 10.924856148387873))
        StdDraw.clear()
        obs.draw()
        bot.draw()
        if (obs.isCollision(bot)) {
            obs.color = Color.RED
            obs.draw()
        } else {
            obs.color = Color.GREEN
            obs.draw()
        }



    }
}