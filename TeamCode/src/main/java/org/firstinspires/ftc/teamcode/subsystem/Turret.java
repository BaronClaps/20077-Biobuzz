package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.utils.Angle.normalizeSigned;

public class Turret {
    private final Servo left, right;
    private double position = .8125;
    private static final double ppr = (0.8125 - .535) / (Math.PI / 2.0);
    public static final double center = .8125;

    public Turret(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "turretLeft");
        right = hardwareMap.get(Servo.class, "turretRight");
    }

    public void position(double position) {
        left.setPosition(position+0.01);
        right.setPosition(position-0.01);
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
        angle(returnFace(targetPose, robotPose));
    }

    public double returnFace(Pose targetPose, Pose robotPose) {
        double angleToTargetFromCenter = Math.atan2(targetPose.y() - robotPose.y(), targetPose.x() - robotPose.x());
        double robotAngleDiff = normalizeSigned(angleToTargetFromCenter - robotPose.heading() + Math.PI/2)-Math.PI/2; // -3PI/2 to PI/2
        robotAngleDiff = Utils.clamp(robotAngleDiff, -3*Math.PI / 2,  Math.PI / 2);
        return robotAngleDiff;
    }
}
