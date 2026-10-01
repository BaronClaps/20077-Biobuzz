package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@Config
@TeleOp(name = "Shooter Test", group = "Tests")
public class ShooterTest extends OpMode {
    Shooter shooter;
    Intake intake;
    public static double velocity = 0, power = 0, mode = 0;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        shooter.disable();
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

        telemetry.addData("Mode", mode == 0 ? "Disabled" : mode == 1 ? "Velocity" : "Power");
        telemetry.addData("Target Velocity", shooter.target());
        telemetry.addData("Current Velocity", shooter.velocity());
        telemetry.addData("Target Power", power);
        telemetry.update();
    }
}
