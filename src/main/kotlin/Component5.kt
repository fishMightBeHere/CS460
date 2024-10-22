import edu.princeton.cs.introcs.StdDraw
import java.awt.Color
import java.util.*
import kotlin.collections.HashSet
import kotlin.math.PI
import kotlin.random.Random

class Component5 {
    fun rrtFB(start: Pair<Matrix, Double>, goal: Pair<Matrix, Double>, env: Environment, bot: Bot,sampleNo:Int  = 1000) {
        bot.teleport(start)
        bot.draw()
        Component1.visualize_scene(env)
        val samples = mutableListOf<SampleFB>()
        val goalSampleFB = SampleFB(-1, goal)
        val startSampleFB = SampleFB(-2, start, cost = 0.0)
        samples.add(startSampleFB)
        for (i in 0..<sampleNo) {
            val smp = SampleFB(
                i,
                Pair(
                    Matrix(Random.nextDouble(-100.0, 100.0), Random.nextDouble(-100.0, 100.0)),
                    Random.nextDouble(0.0, 2 * PI)
                )
            )

            val nn = Component4().nearestNeighborsFB(smp, samples, 60.0, 1)
            if (nn.size > 0 && Component4().collisionFreeFB(bot, smp, nn[0], env)) {
                samples.add(smp)
                nn[0].edges.add(smp)
            }
            println(i)
        }
        // connect closest nodes to goal
        Component4().nearestNeighborsFB(goalSampleFB, samples, 40.0, 1).forEach {
            if (Component4().collisionFreeFB(bot,it,goalSampleFB,env)) {
                it.edges.add(goalSampleFB)
                return@forEach
            }
        }
        samples.add(goalSampleFB)


        StdDraw.setPenColor(Color.ORANGE)
        for (s in samples) {
            StdDraw.filledSquare(s.pose.first[0, 0], s.pose.first[1, 0], 0.5)
            for (l in s.edges) {
                StdDraw.line(s.pose.first[0, 0], s.pose.first[1, 0], l.pose.first[0, 0], l.pose.first[1, 0])
            }
        }
        StdDraw.setPenColor(Color.BLACK)
        StdDraw.filledCircle(start.first[0, 0], start.first[1, 0], 1.0)
        StdDraw.filledCircle(goal.first[0, 0], goal.first[1, 0], 1.0)

        // A*
        val closed = HashSet<SampleFB>()
        val open = PriorityQueue<SampleFB> { a, b ->
            if (a.cost!! + Component4().euclidianDist(a, goalSampleFB) < b.cost!! + Component4().euclidianDist(
                    a,
                    goalSampleFB
                )
            ) -1
            else 1
        }
        val openHS = HashSet<SampleFB>()
        open.add(startSampleFB)
        var cur = startSampleFB
        while (cur != goalSampleFB && open.isNotEmpty()) {
            cur = open.remove()
            closed.add(cur)
            openHS.remove(cur)

            cur.edges.forEach {
                if (it != cur && !closed.contains(it)) {
                    if (it.cost == null) {
                        it.cost = cur.cost!! + Component4().euclidianDist(cur, it)
                        it.cameFrom = cur
                    } else {
                        if (Component4().euclidianDist(cur, it) + cur.cost!! < it.cost!!) {
                            open.remove(it)
                            it.cost = Component4().euclidianDist(cur, it) + cur.cost!!
                            open.add(it)
                            it.cameFrom = cur
                        }
                    }
                }
                if (!openHS.contains(it) && !closed.contains(it)) {
                    open.add(it)
                    openHS.add(it)
                }
            }
        }
        if (cur != goalSampleFB) println("no path found").also { return }

        // draw path
        StdDraw.setPenColor(Color.RED)
        val ll = LinkedList<SampleFB>()
        while (cur != startSampleFB) {
            ll.addFirst(cur)
            StdDraw.line(
                cur.pose.first[0, 0],
                cur.pose.first[1, 0],
                cur.cameFrom!!.pose.first[0, 0],
                cur.cameFrom!!.pose.first[1, 0]
            )
            cur = cur.cameFrom!!
        }
        ll.addFirst(startSampleFB)

        for (i in 0..<ll.size - 1) {
            A1C3.interpolate_rigid_body(
                Matrix(ll.get(i).pose.first[0, 0], ll.get(i).pose.first[1, 0], ll.get(i).pose.second),
                Matrix(ll.get(i + 1).pose.first[0, 0], ll.get(i + 1).pose.first[1, 0], ll.get(i + 1).pose.second)
            ).path.forEach {
                bot.teleport(Pair(Matrix(it.x, it.y), it.theta))
                bot.draw()
            }
        }

        println("complete")
    }

    fun rrtArm(start: Pair<Double, Double>, goal: Pair<Double, Double>, env: Environment, arms: ArmSystem, sampleNo:Int = 1000) {
        arms.teleport(*start.toList().toDoubleArray())
        arms.draw()

        val samples = mutableListOf<SampleArm>()
        Component1.visualize_scene(env)

        val adjustedGoal1 = if (goal.first > PI) goal.first-(2*PI) else if (goal.first < -PI) goal.first+(2*PI) else goal.first
        val adjustedGoal2 = if (goal.second > PI) goal.second-(2*PI) else if (goal.second < -PI) goal.second+(2*PI) else goal.second

        val goalSample = SampleArm(-1, Pair(adjustedGoal1,adjustedGoal2))
        val startSample = SampleArm(-2, start, cost = 0.0)

        samples.add(startSample)
        for (i in 0..<sampleNo) {
            val smp = SampleArm(i, Pair(Random.nextDouble(-2 * PI, 2 * PI), Random.nextDouble(-2 * PI, 2 * PI)))
            val nn = Component4().nearestNeighborsArm(smp, samples, 40.0, 1,5.0)
            if (nn.size > 0 && Component4().collisionFreeArm(arms, smp, nn[0], env)) {
                samples.add(smp)
                nn[0].edges.add(smp)
            }
            println(i)
        }

        Component4().nearestNeighborsArm(goalSample, samples, 40.0, 5,5.0).forEach {
            if (Component4().collisionFreeArm(arms,it,goalSample,env)) {
                it.edges.add(goalSample)
                return@forEach
            }
        }

        // A*
        val closed = HashSet<SampleArm>()
        val open = PriorityQueue<SampleArm> { a, b ->
            if (a.cost!! + Component4().rotDist(a, goalSample) < b.cost!! + Component4().rotDist(a, goalSample)) -1
            else 1
        }
        val openHS = HashSet<SampleArm>()
        var cur = startSample
        open.add(startSample)
        while (cur != goalSample && open.isNotEmpty()) {
            cur = open.remove()
            closed.add(cur)
            openHS.remove(cur)

            cur.edges.forEach {
                if (it != cur && !closed.contains(it)) {
                    if (it.cost == null) {
                        it.cost = cur.cost!! + Component4().rotDist(cur, it)
                        it.cameFrom = cur
                    } else {
                        if (Component4().rotDist(cur, it) + cur.cost!! < it.cost!!) {
                            open.remove(it)
                            it.cost = Component4().rotDist(cur, it) + cur.cost!!
                            open.add(it)
                            it.cameFrom = cur
                        }
                    }
                }
                if (!openHS.contains(it) && !closed.contains(it)) {
                    open.add(it)
                    openHS.add(it)
                }
            }
        }
        if (cur != goalSample) println("no path found").also { return }
        val ll = LinkedList<SampleArm>()
        while (cur != startSample) {
            ll.addFirst(cur)
            cur = cur.cameFrom!!
        }
        ll.addFirst(startSample)

        for (i in 0..<ll.size - 1) {
            A1C4.interpolate_arm(ll[i].pose.toList(), ll[i + 1].pose.toList(), 100).forEach {
                arms.teleport(*it.toDoubleArray())
                arms.draw()
            }
        }
        println("complete")
    }

}

fun main() {
    Drawer(750, 750, 200, 200, 1.1)

    Component5().rrtFB(
        Pair(Matrix(-100.0, -100.0), 0.0), Pair(Matrix(100.0, 100.0), 0.0), Component1.generate_enviroment(27), Bot(
            Pair(Matrix(0.0, 0.0), 0.0),
            mutableListOf(Pair(1.5, -2.5), Pair(-1.5, -2.5), Pair(-1.5, 2.5), Pair(1.5, 2.5)),
            botColor = Color.MAGENTA,
            displayFrame = false
        )
    )

    /*val l1 = Bot(
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
    val arms = ArmSystem(listOf(l1, l2))
    arms.draw()
    arms.teleport(-3*PI/4,0.0)
    arms.draw()
    Component5().rrtArm(
        Pair(0.0, 0.0),
        Pair(-3 * PI / 4, 0.0),
        Environment(obstacles = mutableListOf(SquareObstacle(Pair(Matrix(27.0,0.0),0.0),10.0,5.0))),
        arms
    )*/
}


