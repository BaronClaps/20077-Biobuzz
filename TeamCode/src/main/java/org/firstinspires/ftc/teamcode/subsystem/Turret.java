package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static com.pedropathing.utils.Angle.normalizeSigned;

@Config
public class Turret {
    private final ServoImplEx left, right;
    private double pwm = 1500;
    private static final double ppr = (-1500+1000) / (Math.PI/2);
    public static final double center = 1500;
    public static double seamHysteresisDeg = 5;
    private static final double seamEdge = Math.PI - 1e-3;

    // forward is +, left is +
    public static double pivotForward = -0.59, pivotLeft = 0;

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
        double heading = robotPose.heading();
        double pivotX = robotPose.x() + pivotForward * Math.cos(heading) - pivotLeft * Math.sin(heading);
        double pivotY = robotPose.y() + pivotForward * Math.sin(heading) + pivotLeft * Math.cos(heading);
        double angleToTargetFromCenter = Math.atan2(targetPose.y() - pivotY, targetPose.x() - pivotX);
        double desired = normalizeSigned(angleToTargetFromCenter - robotPose.heading());
        double current = angle();

        boolean crossesSeam = Math.abs(desired - current) > Math.PI;
        if (crossesSeam && Math.PI - Math.abs(desired) < Math.toRadians(seamHysteresisDeg)) {
            return Math.copySign(seamEdge, current);
        }

        return desired;
    }
}
