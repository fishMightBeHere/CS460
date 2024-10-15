import java.awt.Color
import java.util.stream.IntStream
import kotlin.math.PI
import kotlin.random.Random

class Component3 {
    companion object {
        fun collisionChecking(env: Environment) {
            IntStream.range(0, 10).forEach {
                env.bots.add(
                    Bot(
                        Pair(Matrix(0, 0), 0.0),
                        mutableListOf(Pair(15.0, -25.0), Pair(-15.0, -25.0), Pair(-15.0, 25.0), Pair(15.0, 25.0)),
                        botColor = Color.ORANGE
                    ).also { it.teleport(Pair(Matrix(Random.nextDouble(-100.0,100.0)),Random.nextDouble(0.0,2* PI))) }
                )
            }
        }
    }
}