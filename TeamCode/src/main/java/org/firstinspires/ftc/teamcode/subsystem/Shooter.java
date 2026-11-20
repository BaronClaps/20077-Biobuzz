package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Locale;

import static com.pedropathing.ivy.commands.Commands.instant;

@Config
public class Shooter {
    private final CachedMotor bottom;
    private final Servo hood;
    private double target = 0, current = 0, hoodPosition, power;
    public static double kS = 0.035, kV = 0.00034769, kP = 0.005, useRaw = 120, targetThreshold = 20;
    public static double hoodMin = 0.2, hoodMax = 0.5;
    public static double writeThreshold = 0.002, settleMs = 40;
    private long inBandSince = -1;
    public boolean activated = false;

    public Shooter(HardwareMap hardwareMap) {
// top = new CachedMotor(hardwareMap.get(DcMotorEx.class, "shooterTop"), 0.01);
        bottom = new CachedMotor(hardwareMap.get(DcMotorEx.class, "shooterBottom"), writeThreshold);
        bottom.setDirection(DcMotorSimple.Direction.REVERSE);
        hood = hardwareMap.get(Servo.class, "hood");
    }

    public void update() {
        current = bottom.raw().getVelocity();

        if (Math.abs(current - target) <= targetThreshold) {
            if (inBandSince < 0)
                inBandSince = System.nanoTime();
        } else {
            inBandSince = -1;
        }

        if (activated && target > 0) {
//            if (Math.abs(target - velocity()) > useRaw)
//                power(Math.max(Math.signum(target() - velocity()), 0));
//            else
                power(Math.max(0, (kV * target()) + (kP * (target() - velocity())) + kS));
        } else {
            power(0);
        }
    }

    public void power(double power) {
        power = Math.max(0, Math.min(1, power));
// top.setPower(power);
        bottom.setPower(power);
        this.power = power;
    }

    public double power() {
        return power;
    }

    public void target(double target) {
        this.target = target;
    }

    public double target() {
        return target;
    }

    public double velocity() {
        return current;
    }

    public boolean atTarget() {
        return inBandSince >= 0 && System.nanoTime() - inBandSince >= (long) (settleMs * 1e6);
    }

    public void activated(boolean activated) {
        this.activated = activated;
    }

    public void enable() {
        activated = true;
    }

    public void disable() {
        activated = false;
    }

    public void hood(double position) {
        double clamped = Math.max(hoodMin, Math.min(hoodMax, position));
        hood.setPosition(clamped);
        hoodPosition = clamped;
    }

    public double hood() {
        return hoodPosition;
    }

    public void regress(double distance) {
        hood(0.007682 * distance - 0.1493);
        target(6.997 * distance + 1163.1);
    }

    public double current() {
        return bottom.raw().getCurrent(CurrentUnit.AMPS);
    }

    public String currentBreakdown() {
        return String.format(Locale.US, "shooterBottom: %.2fA", bottom.raw().getCurrent(CurrentUnit.AMPS));
    }

    public CommandBuilder enable =instant(this::enable);
    public CommandBuilder disable = instant(this::disable);
}