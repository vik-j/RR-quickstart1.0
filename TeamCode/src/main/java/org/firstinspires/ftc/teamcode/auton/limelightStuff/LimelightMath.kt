package org.firstinspires.ftc.teamcode.auton.limelightStuff

import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit
import org.firstinspires.ftc.teamcode.teleop.Robot

class LimelightMath {
}
fun main() {
    val p1 = Vector2d(-483.73452, 775.61753)
    val p2 = Vector2d(-170.72729, 709.08579)
    val p3 = Vector2d(-629.27271, 90.91421)
    val p4 = Vector2d(-316.26548, 24.38247)

    val angle = Robot.sampleMath(listOf(p1, p2, p3, p4))

    println(angle)
    println(Math.round(Math.toDegrees(Robot.normalizeRadToQ1(angle))))
}