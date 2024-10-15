import edu.princeton.cs.introcs.StdDraw

class Drawer(val xPixels: Int, val yPixels: Int,val xScale: Int = xPixels, val yScale: Int = yPixels,val drawScale:Double = 1.0) {
    init {
        StdDraw.setCanvasSize(xPixels,yPixels)
        StdDraw.setXscale(((-xScale*drawScale)/2).toDouble(), ((xScale*drawScale)/2).toDouble())
        StdDraw.setYscale(((-yScale*drawScale)/2).toDouble(), ((yScale*drawScale)/2).toDouble())
    }

    fun axes() {
        StdDraw.line(0.0,yScale/2.0,0.0,-yScale/2.0)
        StdDraw.line(-xScale/2.0,0.0,xScale/2.0,0.0)
    }
}