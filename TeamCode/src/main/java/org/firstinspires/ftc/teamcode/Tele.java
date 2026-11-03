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
        IN, OUT, STOP, PASSIVE, TRANSFER
    }

    private final Alliance alliance;
    private MultipleTelemetry telemetryM;
    private Robot robot;
    public static Pose start = new Pose(59, 8, Math.toRadians(90));
    public static Pose resetRed = new Pose(7, 6.5+24+2, Math.toRadians(0));
    public static Pose resetBlue = new Pose(141.5-7, 141.5-(6.5+24+2), Math.toRadians(180));
    private IntakeMode intakeMode = IntakeMode.STOP;
    private Timer transferTimer = new Timer();
    private boolean previouslyDetected = false, shooter = true, flower = false, adjustable = true, opened = false;
    public static double targetVelocity = 1440.0, hoodAngle = 0.3, angleOffset = 0.0;

    public Tele(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, alliance);
        robot.pose(start);

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void start() {
        robot.intake.raiseFlower();
        robot.intake.lowerAdjustable();
//        robot.intake.in();
        robot.shooter.enable();
        robot.shooter.hood(hoodAngle);
        robot.intake.close();
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

        if (transferTimer.seconds() > 2.0 && intakeMode == IntakeMode.TRANSFER) {
            intakeMode = IntakeMode.IN;
            robot.intake.close();
        }

        if (transferTimer.seconds() > 0.1 && !opened && intakeMode == IntakeMode.STOP) {
            intakeMode = IntakeMode.TRANSFER;
            opened = true;
        }

        boolean inPressed = gamepad1.leftTriggerWasPressed();
        boolean outPressed = gamepad1.dpadDownWasPressed();

        if (intakeMode != IntakeMode.TRANSFER) {
            if (inPressed) {
                intakeMode = (intakeMode == IntakeMode.IN) ? IntakeMode.STOP : IntakeMode.IN;
            } else if (outPressed) {
                intakeMode = (intakeMode == IntakeMode.OUT) ? IntakeMode.STOP : IntakeMode.OUT;
            }
        }

        switch (intakeMode) {
            case IN:
                robot.intake.in();
                if (robot.loops % 10 == 0) {
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
                break;
            case STOP:
                robot.intake.stop();
                break;
            case PASSIVE:
                robot.intake.passive();
                break;
            case TRANSFER:
                robot.intake.transfer();
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

        if (gamepad1.xWasPressed())
            robot.pose(alliance == Alliance.RED ? resetRed : resetBlue);

        if (gamepad1.dpadLeftWasPressed())
            angleOffset += 3;

        if (gamepad1.dpadRightWasPressed())
            angleOffset -= 3;

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
        telemetryM.addLine(robot.currentBreakdown());
        telemetryM.update();
    }
}
