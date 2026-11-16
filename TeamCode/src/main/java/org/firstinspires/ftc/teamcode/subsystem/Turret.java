package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static com.pedropathing.utils.Angle.normalizeSigned;

public class Turret {
    private final ServoImplEx left, right;
    private double pwm = 2000;
    private static final double ppr = (2000-1500) / (Math.PI / 2.0);
    public static final double center = 2000;

    public Turret(HardwareMap hardwareMap) {
        left = hardwareMap.get(ServoImplEx.class, "turretLeft");
        right = hardwareMap.get(ServoImplEx.class, "turretRight");

        left.setPwmRange(new PwmControl.PwmRange(500,2500));
        right.setPwmRange(new PwmControl.PwmRange(500,2500));

        left.setPwmEnable();
        right.setPwmEnable();
    }

    public void pwm(double pwm) {
        left.setPulseWidth(pwm);
        right.setPulseWidth(pwm);
        this.pwm = pwm;
    }

    public double pwm() {
        return pwm;
    }

    public void manual(double increment) {
        pwm(pwm() + increment);
    }

    public double angle() {
        return normalizeSigned((pwm() - center) / ppr);
    }

    public void angle(double radians) {
        radians = normalizeSigned(radians);
        pwm(center + ppr * radians);
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
