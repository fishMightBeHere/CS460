import edu.princeton.cs.introcs.Draw
import edu.princeton.cs.introcs.StdDraw
import java.awt.Color
import java.util.stream.IntStream
import kotlin.math.PI
import kotlin.random.Random

class Component3 {
    companion object {
        fun collisionChecking(env: Environment) {
            env.bots.add(
                Bot(
                    Pair(Matrix(0.0, 0.0), 0.0),
                    mutableListOf(Pair(1.5, -2.5), Pair(-1.5, -2.5), Pair(-1.5, 2.5), Pair(1.5, 2.5)),
                    botColor = Color.ORANGE
                )
            )
            while (!StdDraw.mousePressed()) {
                env.bots.forEach {
                    it.teleport(
                        Pair(
                            Matrix(Random.nextDouble(-100.0, 100.0), Random.nextDouble(-100.0, 100.0)),
                            Random.nextDouble(0.0, 2 * PI)
                        )
                    )
                }
                env.bots.forEach {
                    for (obstacle in env.obstacles) {
                        if (obstacle.isCollision(it)) obstacle.color = Color.RED
                    }
                    it.draw()
                }
                Component1.visualize_scene(env)
            }

        }
    }
}

fun main() {
    Drawer(750, 750, 200, 200, 0.9)
    val v = Component1.generate_enviroment(10).also { Component1.visualize_scene(it) }
    Component3.collisionChecking(v)

}