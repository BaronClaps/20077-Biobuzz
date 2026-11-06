package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.utils.Angle.normalizeSigned;

public class Turret {
    private final Servo left, right;
    private double position = 0.5;
    private static final double ppr = (.78 - .5) / (Math.PI / 2.0);
    public static final double center = 0.52;
    public static double aimForward = 5;

    public Turret(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "turretLeft");
        right = hardwareMap.get(Servo.class, "turretRight");
    }

    public void position(double position) {
        left.setPosition(position);
        right.setPosition(position);
        this.position = position;
    }

    public double position() {
        return position;
    }

    public void manual(double increment) {
        position(position() + increment);
    }

    public double angle() {
        return normalizeSigned((position() - center) / ppr);
    }

    public void angle(double radians) {
        radians = normalizeSigned(radians);
        position(center + ppr * radians);
    }

    public void face(Pose targetPose, Pose robotPose) {
        double angleToTargetFromCenter = Math.atan2(targetPose.y() - robotPose.y(), targetPose.x() - robotPose.x());
        double robotAngleDiff = normalizeSigned(angleToTargetFromCenter - robotPose.heading());
        robotAngleDiff = Utils.clamp(robotAngleDiff, -2.805, 2.805);
        angle(robotAngleDiff);
    }

    public double returnFace(Pose targetPose, Pose robotPose) {
        double angleToTargetFromCenter = Math.atan2(targetPose.y() - robotPose.y(), targetPose.x() - robotPose.x());
        double robotAngleDiff = normalizeSigned(angleToTargetFromCenter - robotPose.heading());
        robotAngleDiff = Utils.clamp(robotAngleDiff, -2.805, 2.805);
        return robotAngleDiff;
    }

    public void offset(Pose targetPose, Pose robotPose) {
        double h = targetPose.heading();
        Pose aim = new Pose(
                targetPose.x() + aimForward * Math.cos(h),
                targetPose.y() + aimForward * Math.sin(h),
                h);
        face(aim, robotPose);
    }
}
