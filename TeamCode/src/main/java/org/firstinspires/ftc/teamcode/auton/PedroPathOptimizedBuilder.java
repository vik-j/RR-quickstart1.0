//package org.firstinspires.ftc.teamcode.auton;
//
//import android.widget.ArrayAdapter;
//
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class PedroPathOptimizedBuilder {
//    ArrayList<Point> points = new ArrayList<>();
//    ArrayList<BezierCurve> curves = new ArrayList<>();
//
//    public PedroPathOptimizedBuilder() {}
//
//    public BezierCurve buildBezier() {
//        return new BezierCurve(points);
//    }
//    public PathChain buildPathChain() {
//        ArrayList<Path> paths = new ArrayList<>();
//
//        for (BezierCurve curve: curves) {
//            paths.add(new Path(curve));
//        }
//        return new PathChain(paths);
//    }
//}
