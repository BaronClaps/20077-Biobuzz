package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.commands.Commands.instant;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Config
public class Intake {
    public final CachedMotor i, t;
    public final Servo gate, adjustableLeft, adjustableRight, flowerLeft, flowerRight;
//    public final RevColorSensorV3 colorl, colorr;
    public static double inIntakePower = 1, inTransferPower = 0.5, outIntakePower = -1, outTransferPower = -0.5, shootIntakePower = 0.7, shootTransferPower = 0.4, passiveIntakePower = 0.25, passiveTransferPower = 0;
    public static double gateOpen = 0.77, gateClosed = 0.85;
    public static double adjustableDown = 0.52, adjustableUp = 0.24;
    public static double flowerDown = 0.45, flowerUp = 0.7;
    public static double leftThres = 34.0, rightThres = 40.0;

    public Intake(HardwareMap hardwareMap) {
        i = new CachedMotor(hardwareMap.get(DcMotorEx.class, "intake"), 0.01);
        t = new CachedMotor(hardwareMap.get(DcMotorEx.class, "transfer"), 0.01);
        i.setDirection(DcMotorSimple.Direction.REVERSE);
        gate = hardwareMap.get(Servo.class, "gate");
        flowerLeft = hardwareMap.get(Servo.class, "flowerLeft");
        flowerRight = hardwareMap.get(Servo.class, "flowerRight");
        adjustableLeft = hardwareMap.get(Servo.class, "adjLeft");
        adjustableRight = hardwareMap.get(Servo.class, "adjRight");
//        colorl = hardwareMap.get(RevColorSensorV3.class, "colorl");
//        colorr = hardwareMap.get(RevColorSensorV3.class, "colorr");
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

    public void passive() {
        i.setPower(passiveIntakePower);
        t.setPower(passiveTransferPower);
    }

    public void open() {
        gate.setPosition(gateOpen);
    }

    public void close() {
        gate.setPosition(gateClosed);
    }

    public void setFlower(double position) {
        flowerLeft.setPosition(position);
        flowerRight.setPosition(position);
    }

    public void setAdjustable(double position) {
        adjustableLeft.setPosition(position);
        adjustableRight.setPosition(position);
    }

    public void raiseFlower() {
        flowerLeft.setPosition(flowerUp);
        flowerRight.setPosition(flowerUp);
    }

    public void lowerFlower() {
        flowerLeft.setPosition(flowerDown);
        flowerRight.setPosition(flowerDown);
    }

    public void raiseAdjustable() {
        adjustableLeft.setPosition(adjustableUp);
        adjustableRight.setPosition(adjustableUp);
    }

    public void lowerAdjustable() {
        adjustableLeft.setPosition(adjustableDown);
        adjustableRight.setPosition(adjustableDown);
    }

    public double current() {
        return i.raw().getCurrent(CurrentUnit.AMPS) + t.raw().getCurrent(CurrentUnit.AMPS);
    }

    public String currentBreakdown() {
        return String.format(Locale.US, "intake: %.2fA\ntransfer: %.2fA", i.raw().getCurrent(CurrentUnit.AMPS), t.raw().getCurrent(CurrentUnit.AMPS));
    }

    public boolean detected() {
        //return colorl.getDistance(DistanceUnit.MM) < leftThres || colorr.getDistance(DistanceUnit.MM) < rightThres;
        return false;
    }


    public CommandBuilder in = instant(this::in);
    public CommandBuilder out = instant(this::out);
    public CommandBuilder stop = instant(this::stop);
    public CommandBuilder passive = instant(this::passive);
    public CommandBuilder transfer = instant(this::transfer);
    public CommandBuilder open = instant(this::open);
    public CommandBuilder close = instant(this::close);
    public CommandBuilder raiseFlower = instant(this::raiseFlower);
    public CommandBuilder lowerFlower = instant(this::lowerFlower);
    public CommandBuilder raiseAdjustable = instant(this::raiseAdjustable);
    public CommandBuilder lowerAdjustable = instant(this::lowerAdjustable);
}
