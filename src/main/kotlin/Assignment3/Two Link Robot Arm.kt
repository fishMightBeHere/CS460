package Assignment3

import java.io.BufferedReader
import java.io.InputStreamReader

class TwoLinkRobotArm {
    fun generatePath(pathToPython: String, start: Pair<Double,Double>, goal: Pair<Double,Double>, T:Int) : List<Pair<Double,Double>> {
        println("command run")
        println("$pathToPython \"src/main/Python/Assignment3/Two Link Robot.py\" --start ${start.first} ${start.second} --goal ${goal.first} ${goal.second} --T $T")
        val process = Runtime.getRuntime().exec("$pathToPython \"src/main/Python/Assignment3/Two Link Robot.py\" --start ${start.first} ${start.second} --goal ${goal.first} ${goal.second} --T $T")

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

        return output.map{it.split(" ")}.map{Pair(it[0].toDouble(),it[1].toDouble())}.toList()
    }
}
fun main() {
    TwoLinkRobotArm().generatePath()
}