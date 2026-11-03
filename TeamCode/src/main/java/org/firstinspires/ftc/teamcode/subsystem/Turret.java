package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.utils.Angle.normalizeSigned;

public class Turret {
    private final Servo left, right;
    private double position = 0.535;
    private static final double ppr = (.78 - .5) / (Math.PI / 2.0);
    public static final double center = 0.5;
    public static double goalDepth = 10;
    public static double offsetMultiplier = 1.0;
    public static double phiLimit = Math.toRadians(45);

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
        robotAngleDiff = Utils.clamp(robotAngleDiff, -Math.PI, Math.PI);
        angle(robotAngleDiff);
    }

    public double returnFace(Pose targetPose, Pose robotPose) {
        double angleToTargetFromCenter = Math.atan2(targetPose.y() - robotPose.y(), targetPose.x() - robotPose.x());
        double robotAngleDiff = normalizeSigned(angleToTargetFromCenter - robotPose.heading());
        robotAngleDiff = Utils.clamp(robotAngleDiff, -3*Math.PI/4, Math.PI);
        return robotAngleDiff;
    }

    public void offset(Pose targetPose, Pose robotPose) {
        double phi = Utils.clamp(returnFace(robotPose, targetPose), -phiLimit, phiLimit);

        double off = offsetMultiplier * (goalDepth / 2.0) * Math.tan(phi);
        Pose aim = new Pose(
                targetPose.x() - off * Math.sin(targetPose.heading()),
                targetPose.y() + off * Math.cos(targetPose.heading()),
                targetPose.heading());

        angle(returnFace(aim, robotPose));
    }
}