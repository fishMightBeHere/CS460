import edu.princeton.cs.introcs.StdDraw
import java.awt.Color
import java.util.PriorityQueue
import kotlin.math.PI
import kotlin.math.min
import kotlin.math.sqrt
import kotlin.random.Random

data class Sample(
    val id: Int,
    val pose: Pair<Matrix, Double>,
    val edges: HashSet<Sample> = HashSet(),
    var cost: Double? = null,
    var cameFrom: Sample? = null
) {
    override fun hashCode(): Int {
        return id
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as Sample

        return id == other.id
    }

    override fun toString(): String {
        return id.toString()
    }
}

class Component4 {
    fun euclidianDist(p1: Sample, p2: Sample): Double = sqrt(
        (p1.pose.first[0, 0] - p2.pose.first[0, 0]) * (p1.pose.first[0, 0] - p2.pose.first[0, 0]) + (p1.pose.first[1, 0] - p2.pose.first[1, 0]) * (p1.pose.first[1, 0] - p2.pose.first[1, 0])
    )

    //plan is to get this working for free bodies and then generalize to arms
    fun prmFB(start: Pair<Matrix, Double>, goal: Pair<Matrix, Double>, env: Environment, bot: Bot) {
        val samples = mutableListOf<Sample>()
        val ghost = bot // should be bot.clone() deep copy of bot

        Component1.visualize_scene(env)


        for (i in 0..<500) {
            // add samples
            val smp = Sample(
                i, Pair(
                    Matrix(Random.nextDouble(-100.0, 100.0), Random.nextDouble(-100.0, 100.0)),
                    Random.nextDouble(0.0, 2 * PI)
                )
            )
            samples.add(smp)

            //connect samples
            for (p in nearestNeighborsFB(smp.pose, samples, 200.0, 6)) {
                if (collisionFreeFB(ghost, smp, p, env)) {
                    smp.edges.add(p)
                    p.edges.add(smp)
                }
            }
            println(i)
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
        StdDraw.filledCircle(start.first[0,0],start.first[1,0],1.0)
        StdDraw.filledCircle(goal.first[0,0],goal.first[1,0],1.0)
        // A* section
        val closed = HashSet<Sample>()
        // add start and goal as samples
        val goalSample = Sample(-1, goal)
        nearestNeighborsFB(goal, samples, 200.0, 6).filter { collisionFreeFB(bot, goalSample, it, env) }.forEach {
            it.edges.add(goalSample)
            goalSample.edges.add(it)
        }

        val startSample = Sample(-2, start,cost = 0.0)
        nearestNeighborsFB(start, samples, 200.0, 6).filter { collisionFreeFB(bot, it, startSample, env) }.forEach {
            it.edges.add(startSample)
            startSample.edges.add(it)
        }

        val open = PriorityQueue<Sample> { a, b ->
            if (a.cost!! + euclidianDist(a, goalSample) < b.cost!! + euclidianDist(a, goalSample)) -1
            else 1
        }
        val openHS = HashSet<Sample>()


        var cur = startSample
        open.add(startSample)
        while (cur != goalSample && open.isNotEmpty()) {
            cur = open.remove()
            closed.add(cur)
            openHS.remove(cur)

            cur.edges.forEach {
                if (it != cur && !closed.contains(it)) {
                    if (it.cost == null) {
                        it.cost = cur.cost!! + euclidianDist(cur, it)
                        it.cameFrom = cur
                    } else {
                        if (euclidianDist(cur,it) + cur.cost!! < it.cost!!) {
                            open.remove(it)
                            it.cost = euclidianDist(cur,it) + cur.cost!!
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
        // draw path
        StdDraw.setPenColor(Color.RED)
        while (cur != startSample) {
            StdDraw.line(cur.pose.first[0,0],cur.pose.first[1,0], cur.cameFrom!!.pose.first[0,0], cur.cameFrom!!.pose.first[1,0])
            cur = cur.cameFrom!!
        }
        println("complete")
    }

    // search of the nearest neighbors
    private fun nearestNeighborsFB(
        p1: Pair<Matrix, Double>, samples: MutableList<Sample>, r: Double, neighborNumber: Int
    ): MutableList<Sample> {
        val n = mutableListOf<Sample>()
        val d: (Sample) -> Double = { s: Sample ->
            sqrt((p1.first[0, 0] - s.pose.first[0, 0]) * (p1.first[0, 0] - s.pose.first[0, 0]) + (p1.first[1, 0] - s.pose.first[1, 0]) * (p1.first[1, 0] - s.pose.first[1, 0]))
        }

        val comparator = Comparator<Sample> { a, b ->
            if (d(a) < d(b)) -1
            else 1
        }
        val pq = PriorityQueue(comparator)

        pq.addAll(samples) // this is inefficient
        while (n.size <= neighborNumber && pq.size > 0) {
            val sp = pq.remove()
            if (d(sp) < r) n.add(sp)
        }
        return n
    }

    // returns true if there is no collision along path between two points
    fun collisionFreeFB(ghost: Bot, p1: Sample, p2: Sample, env: Environment): Boolean {
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
            botColor = Color.ORANGE
        )
    )
}