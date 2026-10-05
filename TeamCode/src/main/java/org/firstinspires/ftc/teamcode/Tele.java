package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.utils.Alliance;

@Config
public class Tele extends OpMode {

    enum IntakeMode {
        IN, OUT, STOP, PASSIVE, TRANSFER
    }

    private final Alliance alliance;
    private Robot robot;
    public static Pose start = new Pose(141.5-6.5,141.5-8, Math.toRadians(270));
    private IntakeMode intakeMode = IntakeMode.STOP;
    private Timer transferTimer = new Timer();
    private boolean previouslyDetected = false, shooter = true, flower = false, adjustable = true;
    public static double targetVelocity = 1500.0;

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
        robot.shooter.enable();
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
                if (robot.loops % 3 == 0) {
                    boolean detected = robot.intake.detected();
                    if (detected && previouslyDetected) {
                        intakeMode = IntakeMode.PASSIVE;
                        break;
                    }
                    previouslyDetected = detected;
                }
                break;
            case OUT:
                robot.intake.out();
                if (gamepad1.dpadDownWasPressed()) {
                    intakeMode = IntakeMode.STOP;
                    break;
                }
                break;
            case STOP:
                robot.intake.stop();
                if (gamepad1.leftTriggerWasPressed())
                    intakeMode = IntakeMode.IN;
                if (gamepad1.dpadDownWasPressed())
                    intakeMode = IntakeMode.OUT;
                break;
            case PASSIVE:
                if (gamepad1.leftTriggerWasPressed())
                    intakeMode = IntakeMode.STOP;
                robot.intake.passive();
            case TRANSFER:
                robot.intake.transfer();
                break;
        }

        if (gamepad1.rightTriggerWasPressed()) {
            intakeMode = IntakeMode.TRANSFER;
            transferTimer.reset();
            robot.intake.open();
        }

        if (transferTimer.seconds() > 2.0 && intakeMode == IntakeMode.TRANSFER) {
            intakeMode = IntakeMode.IN;
            robot.intake.close();
        }

        if (gamepad1.rightBumperWasPressed()) {
            flower = !flower;
            if (flower)
                adjustable = false;
        }

        if (gamepad1.leftBumperWasPressed())
            adjustable = !adjustable;

        if (gamepad1.bWasPressed())
            shooter = !shooter;

        if (adjustable)
            robot.intake.lowerAdjustable();
        else
            robot.intake.raiseAdjustable();

        if (flower)
            robot.intake.lowerFlower();
        else
            robot.intake.raiseFlower();

        if (shooter) {
            robot.shooter.enable();
            robot.shooter.target(targetVelocity);
        } else {
            robot.shooter.disable();
        }

        robot.turret.face(AimTargets.target(alliance, robot.pose()), robot.pose());
        robot.update();

        telemetry.addData("Pose", robot.pose());
        telemetry.addData("Turret Angle", Math.toDegrees(robot.turret.angle()));
        telemetry.addData("Turret Position", robot.turret.position());
        telemetry.addData("Loop Time HZ", 1000/robot.loopTime);
        telemetry.addData("Intake Mode", intakeMode);
        telemetry.update();
    }
}
