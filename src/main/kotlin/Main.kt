fun main() {
    val m:MutableList<(Int)->Int> = mutableListOf()
    for (i in 0..<5) {
        m.add { it * i }
    }

    m.forEachIndexed { index, function -> println(function(index)) }
}