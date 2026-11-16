package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@Config
@TeleOp(name = "Turret Test", group = "Tests")
public class TurretTest extends OpMode {
    Turret turret;
    public static double angleDegrees = 0, pwm = Turret.center;
    public static boolean isAngle = false;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);

        turret.pwm(Turret.center);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed())
            angleDegrees += 5;

        if (gamepad1.dpadDownWasPressed())
            angleDegrees -= 5;

        if (gamepad1.dpadLeftWasPressed())
            pwm -= 50;

        if (gamepad1.dpadRightWasPressed())
            pwm += 50;

        if (isAngle) {
            turret.angle(Math.toRadians(angleDegrees));
        } else {
            turret.pwm(pwm);
        }

        telemetry.addData("Mode", isAngle ? "Angle" : "Position");
        telemetry.addData("Angle (degrees)", angleDegrees);
        telemetry.addData("Position", pwm);
        telemetry.addData("Current Angle (degrees)", Math.toDegrees(turret.angle()));
        telemetry.addData("Current Position", turret.pwm());
        telemetry.update();
    }
}
