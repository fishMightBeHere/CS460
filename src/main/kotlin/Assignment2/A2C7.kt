package Assignment2

import Bot
import Drawer
import Matrix
import java.awt.Color
import java.awt.Color.BLUE
import kotlin.time.measureTime

class Component7 {
    companion object {
        fun performanceComparisonFB() {
            Drawer(750, 750, 200, 200, 1.1)

            val env: Environment = Component1.generate_enviroment(25)
            Component1.visualize_scene(env)
            val bot = Bot(
                Pair(Matrix(0.0, 0.0), 0.0),
                mutableListOf(Pair(1.5, -2.5), Pair(-1.5, -2.5), Pair(-1.5, 2.5), Pair(1.5, 2.5)),
                botColor = Color.MAGENTA,
                displayFrame = false
            )
            val start = Pair(Matrix(-100.0, -100.0), 0.0)
            val goal = Pair(Matrix(100.0, 100.0), 0.0)
            val prm = measureTime {
                Component4().prmFB(start, goal, env, bot,500)
            }
            bot.botColor = BLUE
            val rrt = measureTime {
                Component5().rrtFB(start, goal, env, bot,500)
            }
            bot.botColor = Color.CYAN
            val prms = measureTime {
                Component6().prmStar(start, goal, env, bot,500)
            }
            println("PRM took $prm")
            println("RRT took $rrt")
            println("PRM* took $prms")
        }
    }
}
fun main() {
    Component7.performanceComparisonFB()
}