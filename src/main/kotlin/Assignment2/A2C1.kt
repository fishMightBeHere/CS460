package Assignment2

import Bot
import Drawer
import Matrix
import SquareObstacle
import java.io.File
import java.io.FileWriter
import kotlin.math.PI
import kotlin.random.Random

data class Environment(
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
    Drawer(750, 750, 200, 200, drawScale = 1.0)
    val a = Component1.generate_enviroment(10);
    Component1.visualize_scene(a);
    Component1.scene_to_file("environment2.txt",a)
    //Component1.visualize_scene(Component1.scene_from_file("C:\\Users\\Marco Hu\\IdeaProjects\\CS460\\environment1.txt"))

}