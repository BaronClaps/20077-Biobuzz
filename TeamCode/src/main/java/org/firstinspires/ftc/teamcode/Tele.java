package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.utils.Alliance;

public class Tele extends OpMode {

    enum IntakeMode {
        IN, OUT, STOP, TRANSFER
    }

    private final Alliance alliance;
    private Robot robot;
    public static Pose start = new Pose(141.5-6.5,141.5-8, Math.toRadians(270));
    private IntakeMode intakeMode = IntakeMode.STOP;

    public Tele(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, alliance);
        robot.pose(start);
    }

    public void start() {
        robot.intake.raiseFlower();
        robot.intake.lowerAdjustable();
//        robot.intake.in();
        robot.intake.close();
        robot.update();
    }

    @Override
    public void loop() {
        robot.follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * .67);

        switch (intakeMode) {
            case IN:
                robot.intake.in();
                if (gamepad1.leftTriggerWasPressed())
                    intakeMode = IntakeMode.STOP;
                break;
            case OUT:
                robot.intake.out();
                break;
            case STOP:
                robot.intake.stop();
                break;
            case TRANSFER:
                robot.intake.transfer();
                break;
        }

        robot.turret.face(AimTargets.target(alliance, robot.pose()), robot.pose());
        robot.update();

        telemetry.addData("Pose", robot.pose());
        telemetry.addData("Turret Angle", Math.toDegrees(robot.turret.angle()));
        telemetry.addData("Turret Position", robot.turret.position());
        telemetry.update();
    }
}
