package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public final CachedMotor intake, transfer;
    public final Servo gate;
    public static final double inIntakePower = 1, inTransferPower = 0.25, outIntakePower = -1, outTransferPower = -1, shootIntakePower = 1, shootTransferPower = 1;
    public static final double gateOpen = 0.5, gateClosed = 0;

    public Intake(HardwareMap hardwareMap) {
        intake = new CachedMotor(hardwareMap.get(DcMotorEx.class, "intake"), 0.01);
        transfer = new CachedMotor(hardwareMap.get(DcMotorEx.class, "transfer"), 0.01);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        gate = hardwareMap.get(Servo.class, "gate");
    }

    public void in() {
        intake.setPower(inIntakePower);
        transfer.setPower(inTransferPower);
    }

    public void out() {
        intake.setPower(outIntakePower);
        transfer.setPower(outTransferPower);
    }

    public void stop() {
        intake.setPower(0);
        transfer.setPower(0);
    }

    public void transfer() {
        intake.setPower(shootIntakePower);
        transfer.setPower(shootTransferPower);
    }

    public void open() {
        gate.setPosition(gateOpen);
    }

    public void close() {
        gate.setPosition(gateClosed);
    }

    public CommandBuilder intake() {
        return Commands.instant(this::in);
    }

    public CommandBuilder outtake() {
        return Commands.instant(this::out);
    }

    public CommandBuilder off() {
        return Commands.instant(this::stop);
    }

    public CommandBuilder shoot() {
        return Commands.instant(this::transfer);
    }

    public CommandBuilder closeGate() {
        return Commands.instant(this::close);
    }

    public CommandBuilder openGate() {
        return Commands.instant(this::open);
    }
}
