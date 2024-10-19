import edu.princeton.cs.introcs.StdDraw
import java.awt.Color
import kotlin.math.PI
import kotlin.random.Random

class Component3 {
    companion object {
        fun collisionCheckingFB(env: Environment) {
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

        fun collisionCheckingArm(env: Environment) {
            val l1 = Bot(
                frame = Pair(Matrix(0.0, 0.0), 0.0),
                points = listOf(Pair(5.0, -10.0), Pair(5.0, 10.0), Pair(-5.0, 10.0), Pair(-5.0, -10.0)),
                endEffector = Pair(Matrix(0.0, 10.0), 0.0),
                botColor = Color.pink,
            )
            val l2 = Bot(
                frame = Pair(Matrix(0.0, 0.0), 0.0),
                points = listOf(Pair(5.0, -1.0), Pair(5.0, 15.0), Pair(-5.0, 15.0), Pair(-5.0, -1.0)),
                endEffector = null,
                botColor = Color.ORANGE,
                root = l1
            )
            Component1.visualize_scene(env)
            val arms = ArmSystem(listOf(l1, l2))
            while (!StdDraw.mousePressed()) {
                arms.teleport(Random.nextDouble(0.0, 2 * PI).also{println(it)}, Random.nextDouble(0.0, 2 * PI).also{println(it)})
                arms.draw()
                env.obstacles.filter { obstacle ->
                    arms.arms.map { arm -> obstacle.isCollision(arm).also {if (obstacle.isCollision(arm)) println("arm at ${arm.frame} has collided with obstacle at ${obstacle.frame}")}
                    }.any { it }
                }.forEach {
                    it.color = Color.RED
                    Component1.visualize_scene(env)
                }
                Thread.sleep(2)
            }
        }
    }
}

fun main() {
    Drawer(750, 750, 200, 200, 0.9)
    val v = Component1.generate_enviroment(10).also { Component1.visualize_scene(it) }
    //Component3.collisionCheckingFB(v)
    Component3.collisionCheckingArm(v)
}