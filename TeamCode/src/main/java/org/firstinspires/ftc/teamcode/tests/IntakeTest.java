package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@TeleOp(name = "Intake Test", group = "Tests")
public class IntakeTest extends OpMode {
    Intake intake;

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

        telemetry.addData("detected?", intake.detected());
        telemetry.addData("hive distance", intake.hiveDistance());
        telemetry.update();
    }
}
