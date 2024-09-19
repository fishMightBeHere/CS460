import java.awt.*
import java.awt.event.WindowAdapter
import java.awt.event.WindowEvent
import kotlin.system.exitProcess

class AwtExample : WindowAdapter() {
    fun run() {
        val frame = Frame("Example")
        val label = Label("Hello")
        label.alignment = Label.CENTER
        frame.add(label)
        frame.setSize(300,300)
        frame.isVisible = true

        frame.addWindowListener(object : WindowAdapter() {
            override fun windowClosing(e: WindowEvent?) {
                exitProcess(0)
            }
        })
    }
}


data class Pose(val x:Double,val y:Double, val theta:Double )

data class Path(val path:MutableList<Pose> = mutableListOf()) //path tmp

class Component3 {
    companion object {
        fun interpolate_rigid_body(start_pose:Matrix, goal_pose:Matrix) : Path {
            val dx = goal_pose[0,0] - start_pose[0,0]
            val dy = goal_pose[1,0] - start_pose[1,0]
            val dth = goal_pose[2,0] - start_pose[2,0]

            val path = Path()

            val steps = 10
            for (i in 0..steps) {
                path.path.add(Pose(start_pose[0,0]+(i*(dx/steps)), start_pose[1,0]+(i*(dy/steps)), start_pose[2,0]+(i*(dth/steps))))
            }

            return path
        }

    }
}

fun main() {
    val v = AwtExample()
    v.run()
}