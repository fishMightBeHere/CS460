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
    }
}