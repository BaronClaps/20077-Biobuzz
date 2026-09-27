package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.commands.Commands.instant;

public class Intake {
    public final CachedMotor i, t;
    public final Servo gate;
    public static final double inIntakePower = 1, inTransferPower = 0.25, outIntakePower = -1, outTransferPower = -1, shootIntakePower = 1, shootTransferPower = 1;
    public static final double gateOpen = 0.5, gateClosed = 0;

    public Intake(HardwareMap hardwareMap) {
        i = new CachedMotor(hardwareMap.get(DcMotorEx.class, "intake"), 0.01);
        t = new CachedMotor(hardwareMap.get(DcMotorEx.class, "transfer"), 0.01);
        t.setDirection(DcMotorSimple.Direction.REVERSE);
        gate = hardwareMap.get(Servo.class, "gate");
    }

    public void in() {
        i.setPower(inIntakePower);
        t.setPower(inTransferPower);
    }

    public void out() {
        i.setPower(outIntakePower);
        t.setPower(outTransferPower);
    }

    public void stop() {
        i.setPower(0);
        t.setPower(0);
    }

    public void transfer() {
        i.setPower(shootIntakePower);
        t.setPower(shootTransferPower);
    }

    public void open() {
        gate.setPosition(gateOpen);
    }

    public void close() {
        gate.setPosition(gateClosed);
    }

    public CommandBuilder in = instant(this::in);
    public CommandBuilder out = instant(this::out);
    public CommandBuilder stop = instant(this::stop);
    public CommandBuilder transfer = instant(this::transfer);
    public CommandBuilder open = instant(this::open);
    public CommandBuilder close = instant(this::close);
}
