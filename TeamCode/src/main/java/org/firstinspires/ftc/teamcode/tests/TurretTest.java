package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Config
@TeleOp(name = "Turret Test", group = "Tests")
public class TurretTest extends OpMode {
    Turret turret;
    public static double angleDegrees = 0, position = 0.5;
    public static boolean isAngle = false;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);

        turret.position(0.5);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed())
            angleDegrees += 5;

        if (gamepad1.dpadDownWasPressed())
            angleDegrees -= 5;

        if (gamepad1.dpadLeftWasPressed())
            position -= 0.05;

        if (gamepad1.dpadRightWasPressed())
            position += 0.05;

        if (isAngle) {
            turret.angle(Math.toRadians(angleDegrees));
        } else {
            turret.position(position);
        }

        telemetry.addData("Mode", isAngle ? "Angle" : "Position");
        telemetry.addData("Angle (degrees)", angleDegrees);
        telemetry.addData("Position", position);
        telemetry.addData("Current Angle (degrees)", Math.toDegrees(turret.angle()));
        telemetry.addData("Current Position", turret.position());
        telemetry.update();
    }
}
