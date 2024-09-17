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



data class Path(val p:Int) //path tmp

class Component3 {
    companion object {
        fun interpolate_rigid_body(start_pose:Matrix, goal_pose:Matrix) : Path {
            return Path(1)
        }

    }
}

fun main() {
    val v = AwtExample()
    v.run()
}