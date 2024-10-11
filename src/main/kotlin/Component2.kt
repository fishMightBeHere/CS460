import java.io.File
import kotlin.math.abs
import kotlin.math.sqrt

/*
* todo
*  add localpose to arm type bots
* */

enum class BotTypes() { FREEBODY, ARM }

class Component2 {
    companion object {
        //calculates Euclidean distance between the two frames
        fun distanceFB(a: Matrix, b: Matrix): Double {
            return sqrt((a[0, 0] - b[0, 0]) * (a[0, 0] - b[0, 0]) + (a[1, 0] - b[1, 0]) * (a[1, 0] - b[1, 0]))
        }

        //calculates sum of the differences in angles of each frame in the arm, a -> [theta1, theta2]
        fun distanceArm(a: Matrix, b: Matrix): Double {
            return abs(a[0, 0] - b[0, 0]) + abs(a[1, 0] - b[1, 0])
        }

        //returns a list of the frames of the nearest neighbors
        fun nearestNeighbors(botType: BotTypes, target: Matrix, k: Int, configs: String): MutableList<Matrix> {
            val ret = mutableListOf<Matrix>()
            File(configs).forEachLine {
                val l = it.split(",").map(String::toDouble).toList()
                if (botType == BotTypes.FREEBODY) {
                    ret.add(Matrix(l[0], l[1], l[2]))
                } else ret.add(Matrix(l[0], l[1]))
            }

            if (botType == BotTypes.FREEBODY) ret.sortBy { distanceFB(target, it) }
            else ret.sortBy { distanceArm(target, it) }

            return ret.subList(0, k)
        }
    }
}