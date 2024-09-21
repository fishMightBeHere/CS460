import edu.princeton.cs.introcs.StdDraw

class Drawer(val x: Int, val y: Int,val drawScale:Int) {
    init {
        StdDraw.setCanvasSize(x,y)
        StdDraw.setXscale(((-x*drawScale)/2).toDouble(), ((x*drawScale)/2).toDouble())
        StdDraw.setYscale(((-y*drawScale)/2).toDouble(), ((y*drawScale)/2).toDouble())
    }

    fun axes() {
        StdDraw.line(0.0,y/2.0,0.0,-y/2.0)
        StdDraw.line(-x/2.0,0.0,x/2.0,0.0)
    }
}