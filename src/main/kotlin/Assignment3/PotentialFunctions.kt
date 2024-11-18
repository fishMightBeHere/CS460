package Assignment3

import Assignment2.Component1
import Assignment2.Environment
import Bot
import Drawer
import Matrix
import java.awt.Color
import java.io.BufferedReader
import java.io.InputStreamReader
import java.util.concurrent.TimeUnit

class PotentialFunctions {
    fun potentialFunctions(pathToPython:String, start: Pair<Matrix, Double>, goal: Pair<Matrix, Double>, env: String) {
        println("$pathToPython \"src/main/Python/Assignment3/Potential Functions.py\" $env \"${start.first[0,0]};${start.first[1,0]};${start.second}\" \"${goal.first[0,0]};${goal.first[1,0]};${goal.second}\"")
        val process = Runtime.getRuntime().exec("$pathToPython \"src/main/Python/Assignment3/Potential Functions.py\" $env \"${start.first[0,0]};${start.first[1,0]};${start.second}\" \"${goal.first[0,0]};${goal.first[1,0]};${goal.second}\"")

        val reader = BufferedReader(InputStreamReader(process.inputStream))
        val output = mutableListOf<String>()
        var l:String?
        while (reader.readLine().also { l = it } != null) {
            output.add(l!!)
        }

        val error = BufferedReader(InputStreamReader(process.errorStream))
        while (error.readLine() != null) {
            println(output)
        }

        output.forEach { println(it) }
        process.destroy()

        Drawer(750,750,200,200,1.0)
        val bot = Bot(
            Pair(Matrix(0.0, 0.0), 0.0),
            mutableListOf(Pair(1.5, -2.5), Pair(-1.5, -2.5), Pair(-1.5, 2.5), Pair(1.5, 2.5)),
            botColor = Color.MAGENTA,
            displayFrame = false
        )
        output.removeFirst()
        Component1.visualize_scene(Component1.scene_from_file("environment1.txt"))
        output.map{it.split(" ").map{ i -> i.toDouble()}.toDoubleArray()}.map{Pair(Matrix(it[0],it[1]),it[2])}.forEach {
            bot.teleport(it)
            bot.draw()
        }

    }
}

fun main() {
    PotentialFunctions().potentialFunctions(
        "D:\\anaconda3\\envs\\CS460A3\\python",
        Pair(Matrix(-100.0, -100.0), 0.0),
        Pair(Matrix(100.0, 100.0), 0.0),
        "environment1.txt"
    )
}