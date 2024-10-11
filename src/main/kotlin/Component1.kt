import edu.princeton.cs.introcs.Draw
import java.io.File
import java.io.FileReader
import java.io.FileWriter
import kotlin.math.PI
import kotlin.random.Random

class Environment(
    val obstacles: MutableList<SquareObstacle> = mutableListOf(),
    val bots: MutableList<Bot> = mutableListOf()
)

class Component1 {
    companion object {
        fun generate_enviroment(number_of_obstacles: Int): Environment {
            val ev = Environment();
            for (i in 0..<number_of_obstacles) {
                ev.obstacles.add(
                    SquareObstacle(
                        Pair(
                            Matrix(Random.nextDouble(-100.0, 100.0), Random.nextDouble(-100.0, 100.0)),
                            Random.nextDouble(0.0, 2 * PI)
                        ),
                        Random.nextDouble(5.0, 20.0),
                        Random.nextDouble(5.0, 20.0)
                    )
                )
            }
            return ev;
        }

        fun scene_to_file(fileName: String, env: Environment) {
            val fw = FileWriter(fileName)
            for (i in env.obstacles) {
                // x, y, theta, l, w
                fw.append("${i.frame.first[0][0]},${i.frame.first[1][0]},${i.frame.second},${i.length},${i.width}\n")
            }
            fw.close()
        }

        fun scene_from_file(fileName: String): Environment {
            val env = Environment()
            val file = File(fileName)
            file.forEachLine {
                val i: List<Double> = it.split(",").map { v -> v.toDouble() }.toList()
                env.obstacles.add(SquareObstacle(Pair(Matrix(i[0], i[1]), i[2]), i[3], i[4]))
            }
            return env
        }

        fun visualize_scene(env: Environment) {
            env.obstacles.forEach(SquareObstacle::draw)
            env.bots.forEach(Bot::draw)
        }
    }
}

fun main() {
    val b = Drawer(750, 750, 0.3)
    val a = Component1.generate_enviroment(5);
    Component1.visualize_scene(a);
}