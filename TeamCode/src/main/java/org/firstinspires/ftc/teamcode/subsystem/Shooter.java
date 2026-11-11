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
    private double target = 0, current = 0, hoodPosition;
    public static double kS = 0.106135, kV = 0.00038213, kP = 0.025, useRaw = 60, targetThreshold = 20;
    public static double hoodMin = 0.2, hoodMax = 0.5;
    public boolean activated = false;

    public Shooter(HardwareMap hardwareMap) {
//        top = new CachedMotor(hardwareMap.get(DcMotorEx.class, "shooterTop"), 0.01);
        bottom = new CachedMotor(hardwareMap.get(DcMotorEx.class, "shooterBottom"), 0.01);
        bottom.setDirection(DcMotorSimple.Direction.REVERSE);
        hood = hardwareMap.get(Servo.class, "hood");
    }

    public void update() {
        current = bottom.raw().getVelocity();

        if (activated) {
            if (Math.abs(target - velocity()) > useRaw)
                power(Math.max(Math.signum(target() - velocity()), 0));
            else
                power(Math.max(0, (kV * target()) + (kP * (target() - velocity())) + kS));
        } else {
            power(0);
        }
    }

    public void power(double power) {
        // top.setPower(power);
        bottom.setPower(power);
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
        return Math.abs(current - target) <= targetThreshold;
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
