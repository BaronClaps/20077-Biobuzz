package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Localization Test", group = "Tests")
public class LocalizationTest extends OpMode {
    Follower follower;
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        follower.setPose(Pose.zero());
    }

    public void start() {
        follower.setPose(Pose.zero());
        follower.update();
    }

    @Override
    public void loop() {
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();

        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }
}
