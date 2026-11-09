package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Config
@TeleOp(name = "Shooter Test", group = "Tests")
public class ShooterTest extends OpMode {
    Shooter shooter;
    Intake intake;
    Turret turret;
    MultipleTelemetry telemetryM;
    public static double velocity = 1520, power = 0, mode = 0, hood = 0.45;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);

        turret.angle(0);

        intake.raiseAdjustable();
        intake.lowerFlower();
        intake.open();

        shooter.disable();
        shooter.hood(0.45);

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed())
            velocity += 20;

        if (gamepad1.dpadDownWasPressed())
            velocity -= 20;

        if (gamepad1.dpadLeftWasPressed())
            power -= 0.1;

        if (gamepad1.dpadRightWasPressed())
            power += 0.1;

        if (gamepad1.rightTriggerWasPressed())
            hood += 0.05;

        if (gamepad1.leftTriggerWasPressed())
            hood -= 0.05;

        shooter.hood(hood);
        hood = shooter.hood();

        if (mode == 0) {
            shooter.disable();
        } else if (mode == 1) {
            shooter.enable();
            shooter.target(velocity);
        } else if (mode == 2) {
            shooter.disable();
            shooter.power(power);
        }

        if (gamepad1.aWasPressed())
            mode = 0;

        if (gamepad1.bWasPressed())
            mode = 1;

        if (gamepad1.xWasPressed())
            mode = 2;

        if (gamepad1.leftBumperWasPressed())
            intake.stop();

        if (gamepad1.rightBumperWasPressed())
            intake.transfer();

        shooter.update();

        telemetryM.addData("Mode", mode == 0 ? "Disabled" : mode == 1 ? "Velocity" : "Power");
        telemetryM.addData("Target Velocity", shooter.target());
        telemetryM.addData("Current Velocity", shooter.velocity());
        telemetryM.addData("Target Power", power);
        telemetryM.addData("Hood Position", shooter.hood());
        telemetryM.update();
    }
}
