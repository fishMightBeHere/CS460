import edu.princeton.cs.introcs.StdDraw
import java.awt.Color
import java.util.LinkedList
import java.util.PriorityQueue
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.random.Random

data class SampleFB(
    val id: Int,
    val pose: Pair<Matrix, Double>,
    val edges: HashSet<SampleFB> = HashSet(),
    var cost: Double? = null,
    var cameFrom: SampleFB? = null
) {
    override fun hashCode(): Int {
        return id
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as SampleFB

        return id == other.id
    }

    override fun toString(): String {
        return id.toString()
    }
}

data class SampleArm(
    val id: Int,
    val pose: Pair<Double, Double>,
    val edges: HashSet<SampleArm> = HashSet(),
    var cost: Double? = null,
    var cameFrom: SampleArm? = null
) {
    override fun hashCode(): Int {
        return id
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as SampleArm

        return id == other.id
    }

    override fun toString(): String {
        return id.toString()
    }
}

class Component4 {
    fun euclidianDist(p1: SampleFB, p2: SampleFB): Double = sqrt(
        (p1.pose.first[0, 0] - p2.pose.first[0, 0]) * (p1.pose.first[0, 0] - p2.pose.first[0, 0]) + (p1.pose.first[1, 0] - p2.pose.first[1, 0]) * (p1.pose.first[1, 0] - p2.pose.first[1, 0])
    )

    // sum of the differences in angles of arm
    fun rotDist(p1: SampleArm, p2: SampleArm): Double =
        abs(p1.pose.first - p2.pose.first) % (2 * PI) + abs(p1.pose.second - p2.pose.second) % (2 * PI)

    //plan is to get this working for free bodies and then generalize to arms
    fun prmFB(start: Pair<Matrix, Double>, goal: Pair<Matrix, Double>, env: Environment, bot: Bot) {
        val sampleFBS = mutableListOf<SampleFB>()
        val ghost = bot // should be bot.clone() deep copy of bot

        Component1.visualize_scene(env)
        val goalSampleFB = SampleFB(-1, goal)
        val startSampleFB = SampleFB(-2, start, cost = 0.0)

        for (i in 0..<500) {
            // add samples
            val smp = SampleFB(
                i, Pair(
                    Matrix(Random.nextDouble(-100.0, 100.0), Random.nextDouble(-100.0, 100.0)),
                    Random.nextDouble(0.0, 2 * PI)
                )
            )
            sampleFBS.add(smp)

            //connect samples
            for (p in nearestNeighborsFB(smp, sampleFBS, 50.0, 6)) {
                if (collisionFreeFB(ghost, smp, p, env)) {
                    smp.edges.add(p)
                    p.edges.add(smp)
                }
            }
            println("Sample : $i")
        }

        // draw the tree
        for (s in sampleFBS) {
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

        nearestNeighborsFB(goalSampleFB, sampleFBS, 50.0, 6).filter { collisionFreeFB(bot, goalSampleFB, it, env) }.forEach {
            it.edges.add(goalSampleFB)
            goalSampleFB.edges.add(it)
        }
        nearestNeighborsFB(startSampleFB, sampleFBS, 50.0, 6).filter { collisionFreeFB(bot, it, startSampleFB, env) }.forEach {
            it.edges.add(startSampleFB)
            startSampleFB.edges.add(it)
        }
        val open = PriorityQueue<SampleFB> { a, b ->
            if (a.cost!! + euclidianDist(a, goalSampleFB) < b.cost!! + euclidianDist(a, goalSampleFB)) -1
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
                        it.cost = cur.cost!! + euclidianDist(cur, it)
                        it.cameFrom = cur
                    } else {
                        if (euclidianDist(cur, it) + cur.cost!! < it.cost!!) {
                            open.remove(it)
                            it.cost = euclidianDist(cur, it) + cur.cost!!
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

    // search of the nearest neighbors
    private fun  nearestNeighborsFB(
        p1: SampleFB, samples: MutableList<SampleFB>, r: Double, neighborNumber: Int
    ): MutableList<SampleFB> {
        val n = mutableListOf<SampleFB>()

        val comparator = Comparator<SampleFB> { a, b ->
            if (euclidianDist(a,p1) < euclidianDist(b,p1)) -1
            else 1
        }

        val pq = PriorityQueue(comparator)
        pq.addAll(samples) // this is inefficient
        while (n.size <= neighborNumber && pq.size > 0) {
            val sp = pq.remove()
            if (euclidianDist(sp,p1) < r) n.add(sp)
        }
        return n
    }

    // returns true if there is no collision along path between two points
    fun collisionFreeFB(ghost: Bot, p1: SampleFB, p2: SampleFB, env: Environment): Boolean {
        ghost.teleport(p1.pose)
        return !A1C3.interpolate_rigid_body(
            Matrix(p1.pose.first[0, 0], p1.pose.first[1, 0], p1.pose.second),
            Matrix(p2.pose.first[0, 0], p2.pose.first[1, 0], p2.pose.second)
        ).path.map { pose ->
            ghost.teleport(Pair(Matrix(pose.x, pose.y), pose.theta))
            env.obstacles.map { obstacle -> obstacle.isCollision(ghost) }.any { it }
        }.any { it }
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
}