import edu.princeton.cs.introcs.StdDraw
import java.awt.Color
import java.lang.Math.log
import java.util.*
import kotlin.collections.HashSet
import kotlin.math.E
import kotlin.math.PI
import kotlin.math.ceil
import kotlin.math.ln
import kotlin.random.Random

class Component6 {
    fun prmStar(start: Pair<Matrix, Double>, goal: Pair<Matrix, Double>, env: Environment, bot: Bot,sampleNo:Int = 1000) {
        val samples = mutableListOf<SampleFB>()
        val ghost = bot // should be bot.clone() deep copy of bot

        Component1.visualize_scene(env)
        val goalSampleFB = SampleFB(-1, goal)
        val startSampleFB = SampleFB(-2, start, cost = 0.0)

        for (i in 0..<sampleNo) {
            // add samples
            val smp = SampleFB(
                i, Pair(
                    Matrix(Random.nextDouble(-100.0, 100.0), Random.nextDouble(-100.0, 100.0)),
                    Random.nextDouble(0.0, 2 * PI)
                )
            )

            samples.add(smp)
            //connect samples
            for (p in Component4().nearestNeighborsFB(smp, samples, 50.0, ceil(E * (1 + (1 / 3)) * ln(3.0)).toInt())) {
                if (Component4().collisionFreeFB(ghost, smp, p, env)) {

                    smp.edges.add(p)
                    p.edges.add(smp)
                }
            }
            println("Sample : $i")
        }

        // draw the tree
        for (s in samples) {
            StdDraw.setPenColor(Color.ORANGE)
            StdDraw.filledSquare(s.pose.first[0, 0], s.pose.first[1, 0], 0.5)
            for (l in s.edges) {
                StdDraw.line(s.pose.first[0, 0], s.pose.first[1, 0], l.pose.first[0, 0], l.pose.first[1, 0])
            }
        }
        StdDraw.setPenColor(Color.BLACK)
        StdDraw.filledCircle(start.first[0, 0], start.first[1, 0], 1.0)
        StdDraw.filledCircle(goal.first[0, 0], goal.first[1, 0], 1.0)

        // A* section
        val closed = HashSet<SampleFB>()
        // add start and goal as samples

        Component4().nearestNeighborsFB(goalSampleFB, samples, 50.0, ceil(E * (1 + (1 / 3)) * ln(3.0)).toInt())
            .filter { Component4().collisionFreeFB(bot, goalSampleFB, it, env) }
            .forEach {
                it.edges.add(goalSampleFB)
                goalSampleFB.edges.add(it)
            }
        Component4().nearestNeighborsFB(startSampleFB, samples, 50.0, ceil(E * (1 + (1 / 3)) * ln(3.0)).toInt())
            .filter { Component4().collisionFreeFB(bot, it, startSampleFB, env) }
            .forEach {
                it.edges.add(startSampleFB)
                startSampleFB.edges.add(it)
            }
        val open = PriorityQueue<SampleFB> { a, b ->
            if (a.cost!! + Component4().euclidianDist(a, goalSampleFB) < b.cost!! + Component4().euclidianDist(
                    b,
                    goalSampleFB
                )
            ) -1
            else 1
        }
        val openHS = HashSet<SampleFB>()

        var cur = startSampleFB
        open.add(startSampleFB)
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

    //hoping that we won't need 3 segment arms because this will need to get rewritten
    fun prmArm(start: Pair<Double, Double>, goal: Pair<Double, Double>, env: Environment, arms: ArmSystem, sampleNo:Int = 1000) {
        val samples = mutableListOf<SampleArm>()
        val ghost = arms

        Component1.visualize_scene(env)
        val adjustedGoal1 =
            if (goal.first > PI) goal.first - (2 * PI) else if (goal.first < -PI) goal.first + (2 * PI) else goal.first
        val adjustedGoal2 =
            if (goal.second > PI) goal.second - (2 * PI) else if (goal.second < -PI) goal.second + (2 * PI) else goal.second

        val goalSample = SampleArm(-1, Pair(adjustedGoal1, adjustedGoal2))
        val startSample = SampleArm(-2, start, cost = 0.0)

        for (i in 0..<sampleNo) {
            val smp = SampleArm(i, Pair(Random.nextDouble(-2 * PI, 2 * PI), Random.nextDouble(-2 * PI, 2 * PI)))

            samples.add(smp)
            for (p in Component4().nearestNeighborsArm(smp, samples, 60.0, ceil(E * (1 + (1 / 2)) * ln(2.0)).toInt())) {
                if (Component4().collisionFreeArm(ghost, smp, p, env)) {

                    smp.edges.add(p)
                    p.edges.add(smp)
                }
            }
            println("Sample : $i")
        }

        // Due to the fact that it is hard to easily represent poses we neglect this part for now...

        // A*
        val closed = HashSet<SampleArm>()
        Component4().nearestNeighborsArm(goalSample, samples, 60.0, ceil(E * (1 + (1 / 2)) * ln(2.0)).toInt())
            .filter { Component4().collisionFreeArm(arms, goalSample, it, env) }
            .forEach {
                it.edges.add(goalSample)
                goalSample.edges.add(it)
            }
        Component4().nearestNeighborsArm(startSample, samples, 60.0, ceil(E * (1 + (1 / 2)) * ln(2.0)).toInt())
            .filter { Component4().collisionFreeArm(arms, startSample, it, env) }
            .forEach {
                it.edges.add(startSample)
                startSample.edges.add(it)
            }

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

    Component4().prmFB(
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
    arms.teleport(5*PI/4, 0.0)
    arms.draw()
    arms.draw()
    Component4().prmArm(
        Pair(0.0, 0.0),
        Pair(5 * PI / 4, 0.0),
        Environment(obstacles = mutableListOf(SquareObstacle(Pair(Matrix(25.0,0.0),0.0),10.0,5.0))),
        arms
    )*/
}