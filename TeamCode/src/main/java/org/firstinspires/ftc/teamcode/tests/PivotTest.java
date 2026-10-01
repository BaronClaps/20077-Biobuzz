package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@Config
@TeleOp(name = "Pivot Test", group = "Tests")
public class PivotTest extends OpMode {
    Intake intake;

    public static double adjustable = 0.5, flower = 0.5;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);

        intake.close();
        intake.stop();
    }

    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed())
            intake.in();
        else if (gamepad1.leftBumperWasPressed())
            intake.out();
        else if (gamepad1.aWasPressed())
            intake.stop();
        else if (gamepad1.bWasPressed())
            intake.transfer();
        else if (gamepad1.rightTriggerWasPressed())
            intake.open();
        else if (gamepad1.leftTriggerWasPressed())
            intake.close();

        intake.setFlower(flower);
        intake.setAdjustable(adjustable);
    }
}
