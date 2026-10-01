package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.commands.Commands.instant;

@Config
public class Shooter {
    private final CachedMotor top, bottom;
    private double target = 0, current = 0;
    public static double kS = 0.129403, kV = 0.00042071, kP = 0.00705, useRaw = 80, targetThreshold = 40;
    public boolean activated = false;

    public Shooter(HardwareMap hardwareMap) {
        top = new CachedMotor(hardwareMap.get(DcMotorEx.class, "shooterTop"), 0.01);
        bottom = new CachedMotor(hardwareMap.get(DcMotorEx.class, "shooterBottom"), 0.01);
        top.setDirection(DcMotorSimple.Direction.REVERSE);
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
        top.setPower(power);
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
        return Math.abs(current - target) < targetThreshold;
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

    public CommandBuilder enable = instant(this::enable);
    public CommandBuilder disable = instant(this::disable);
}
