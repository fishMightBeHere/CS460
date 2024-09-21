import edu.princeton.cs.introcs.StdDraw

class Drawer(x: Int, y: Int, drawScale:Int) {
    init {
        StdDraw.setCanvasSize(x,y)
        StdDraw.setXscale(((-x*drawScale)/2).toDouble(), ((x*drawScale)/2).toDouble())
        StdDraw.setYscale(((-y*drawScale)/2).toDouble(), ((y*drawScale)/2).toDouble())

    }
}