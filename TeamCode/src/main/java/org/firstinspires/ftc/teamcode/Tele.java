package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.utils.Alliance;

@Config
public class Tele extends OpMode {

    enum IntakeMode {
        IN, OUT, STOP, SEMI, PASSIVE, TRANSFER
    }

    private final Alliance alliance;
    private MultipleTelemetry telemetryM;
    private Robot robot;
    public static Pose start = new Pose(59, 8, Math.toRadians(90));
    public static Pose resetRed = new Pose(7, 6.5+24+2, Math.toRadians(0));
    public static Pose resetBlue = new Pose(141.5-7, 141.5-(6.5+24+2), Math.toRadians(180));
    private IntakeMode intakeMode = IntakeMode.STOP;
    private Timer transferTimer = new Timer();
    private boolean previouslyDetected = false, previouslySemi = false, shooter = false, flower = false, adjustable = true, opened = true, rejectSequence = false;
    public static double targetVelocity = 1420.0, hoodAngle = 0.265, angleOffset = 0.0;

    public Tele(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, alliance);
        robot.pose(start);

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed())
            robot.follower.localizer.reset();
    }

    public void start() {
        robot.intake.raiseFlower();
        robot.intake.lowerAdjustable();
//        robot.intake.in();
        robot.shooter.enable();
        intakeMode = IntakeMode.STOP;
        robot.shooter.hood(hoodAngle);
        robot.intake.close();
        robot.intake.stop();
        robot.update();
    }

    @Override
    public void loop() {
        robot.follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * .67);

        if (gamepad1.rightTriggerWasPressed()) {
            transferTimer.reset();
            robot.intake.open();
            intakeMode = IntakeMode.STOP;
            opened = false;
        }

        if (transferTimer.seconds() > 1.2 && intakeMode == IntakeMode.TRANSFER) {
            intakeMode = IntakeMode.IN;
            robot.intake.close();
        }

        if (transferTimer.seconds() > 1 && intakeMode == IntakeMode.OUT && rejectSequence) {
            intakeMode = IntakeMode.TRANSFER;
            rejectSequence = false;
        }

        if (transferTimer.seconds() > 0.85 && intakeMode == IntakeMode.STOP && rejectSequence) {
            intakeMode = IntakeMode.OUT;
        }

        if (transferTimer.seconds() > 0.7 && intakeMode == IntakeMode.TRANSFER && rejectSequence) {
            intakeMode = IntakeMode.STOP;
        }

        if (transferTimer.seconds() > 0.1 && !opened && intakeMode == IntakeMode.STOP) {
            rejectSequence = true;
            intakeMode = IntakeMode.TRANSFER;
            opened = true;
        }

        if (gamepad1.dpadUpWasPressed())
            hoodAngle -= 0.005;

        if (gamepad1.backWasPressed())
            robot.follower.localizer.reset();

        boolean inPressed = gamepad1.leftTriggerWasPressed();
        boolean outPressed = gamepad1.dpadDownWasPressed();

        if (intakeMode != IntakeMode.TRANSFER) {
            if (inPressed) {
                intakeMode = ((intakeMode == IntakeMode.IN || intakeMode == IntakeMode.SEMI || intakeMode == IntakeMode.PASSIVE)) ? IntakeMode.STOP : IntakeMode.IN;
            } else if (outPressed) {
                intakeMode = (intakeMode == IntakeMode.OUT) ? IntakeMode.STOP : IntakeMode.OUT;
            }
        }

        switch (intakeMode) {
            case IN:
                robot.intake.in();
                robot.light.in();
                check();
                break;
            case OUT:
                robot.intake.out();
                robot.light.out();
                break;
            case STOP:
                robot.intake.stop();
                robot.light.stop();
                break;
            case PASSIVE:
                robot.intake.passive();
                robot.light.ready();
                break;
            case TRANSFER:
                robot.intake.transfer();
                robot.light.shoot();
                break;
            case SEMI:
                robot.intake.semi();
                robot.light.in();
                check();
                break;
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
            robot.shooter.hood(hoodAngle);
        } else {
            robot.shooter.disable();
        }

        if (gamepad1.xWasPressed()) {
            robot.pose(alliance == Alliance.RED ? resetRed : resetBlue);
            angleOffset = 0;
        }

        if (gamepad1.dpadLeftWasPressed())
            angleOffset += 5;

        if (gamepad1.dpadRightWasPressed())
            angleOffset -= 5;

        if (gamepad1.yWasPressed())
            targetVelocity += 20;

        if (gamepad1.aWasPressed())
            targetVelocity -= 20;

        robot.turret.angle(robot.turret.returnFace(AimTargets.target(alliance, robot.pose()), robot.pose()) + Math.toRadians(angleOffset));
        robot.update();

        telemetryM.addData("Pose", robot.pose());
        telemetryM.addData("Turret Angle", Math.toDegrees(robot.turret.angle()));
        telemetryM.addData("Turret Position", robot.turret.position());
        telemetryM.addData("Loop Time HZ", 1000 / robot.loopTime);
        telemetryM.addData("Intake Mode", intakeMode);
        telemetryM.addData("Shooter Velocity", robot.shooter.velocity());
        telemetryM.addData("Distance from Target", AimTargets.target(alliance, robot.pose()).distance(robot.pose()));
        telemetryM.addData("X & Y from Target", AimTargets.target(alliance, robot.pose()).minus(robot.pose()));
//        telemetryM.addLine(robot.currentBreakdown());
        telemetryM.update();
    }

    public void stop() {
        robot.light.off();
        super.stop();
    }

    public void check() {
        if (robot.loops % 20 == 0) {
            if (intakeMode == IntakeMode.PASSIVE)
                return;

            boolean detected = robot.intake.detected();
            if (detected && previouslyDetected) {
                intakeMode = IntakeMode.PASSIVE;
                return;
            }
            previouslyDetected = detected;

            if (intakeMode == IntakeMode.SEMI)
                return;

            boolean semi = robot.intake.transferStalling();
            if (semi && previouslySemi) {
                intakeMode = IntakeMode.SEMI;
                return;
            }
            previouslySemi = semi;
        }
    }
}
