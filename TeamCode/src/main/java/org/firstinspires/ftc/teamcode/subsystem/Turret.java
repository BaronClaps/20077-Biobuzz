package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static com.pedropathing.utils.Angle.normalizeSigned;

public class Turret {
    private final ServoImplEx left, right;
    private double pwm = 1500;
    private static final double ppr = (2475-2000) / (Math.PI/2);
    public static final double center = 1500;

    public Turret(HardwareMap hardwareMap) {
        left = hardwareMap.get(ServoImplEx.class, "turretLeft");
        right = hardwareMap.get(ServoImplEx.class, "turretRight");

        left.setPwmRange(new PwmControl.PwmRange(499,2501));
        right.setPwmRange(new PwmControl.PwmRange(499,2501));

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
        double robotAngleDiff = normalizeSigned(angleToTargetFromCenter - robotPose.heading());
        robotAngleDiff = Utils.clamp(robotAngleDiff, -Math.PI,  Math.PI);
        return robotAngleDiff;
    }
}
