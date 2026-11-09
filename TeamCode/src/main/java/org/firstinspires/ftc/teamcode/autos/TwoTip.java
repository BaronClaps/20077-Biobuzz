package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tele;
import org.firstinspires.ftc.teamcode.autos.TwoTipPaths;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CommandOpMode;

public class TwoTip extends CommandOpMode {
    final Alliance alliance;
    TwoTipPaths paths;
    Robot robot;

    public TwoTip(Alliance alliance) {
        this.alliance = alliance;
    }

    public void init() {
        robot = new Robot(hardwareMap, alliance);
        paths = new TwoTipPaths(robot.follower, alliance);

        robot.pose(paths.start());
        robot.shooter.disable();
        robot.turret.angle(Math.toRadians(-113));
        robot.intake.close();
        robot.intake.stop();
        robot.intake.lowerAdjustable();
        robot.intake.raiseFlower();

        schedule(
                sequential(
                        setup(1420, 0.26, -113),
                        instant(() -> robot.pose(paths.start())),
                        waitUntil(() -> robot.shooter.atTarget()),
                        waitMs(500),
                        robot.shoot(),
//                        robot.shoot(),
                        robot.intake(),
                        robot.intake.lowerAdjustable,
                        paths.spike()
                                .raceWith(waitMs(2000)),
                        waitMs(250),
                        paths.spikeShoot()
                                .raceWith(waitMs(2000)),
                        robot.shoot(),
                        robot.intake.raiseAdjustable,
                        robot.intake.open,
                        robot.intake.stop,
                        robot.intake.raiseFlower,
                        setup(1000, 0.265, 38),
                        paths.flower1()
                                .with(
                                        waitUntil(() -> robot.follower.completion() > 0.8)
                                                .then(
                                                        robot.intake.raiseAdjustable,
                                                        robot.intake.lowerFlower,
                                                        robot.intake.close,
                                                        robot.intake.in
                                                )
                                )
                                .raceWith(waitMs(5000)),
                        setup(1420, 0.265, 36.5),
                        waitMs(2000),
                        paths.flower1Shoot()
                                .raceWith(waitMs(2000)),
                        robot.intake.lowerAdjustable,
                        robot.shoot(),
                        robot.shoot(),
                        robot.intake.raiseAdjustable,
                        robot.intake(),
                        paths.flower2()
                                .with(
                                        waitUntil(() -> robot.follower.completion() > 0.2)
                                                .then(
                                                        robot.intake.raiseAdjustable
                                                ),
                                        waitUntil(() -> robot.follower.completion() > 0.8)
                                                .then(
                                                        robot.intake.raiseAdjustable,
                                                        robot.intake.lowerFlower
                                                )
                                )
                                .raceWith(waitMs(5000)),
                        waitMs(2000),
                        setup(1420, 0.265, 38),
                        paths.score()
                                .with(
                                        waitUntil(() -> robot.follower.completion() > 0.2)
                                                .then(
                                                        robot.intake.raiseAdjustable
                                                )
                                )
                                .raceWith(waitMs(5000)),
                        robot.intake.lowerAdjustable,
                        robot.shoot(),
//                        robot.shoot(),
                        paths.park()
                                .with(
                                        robot.intake.raiseFlower,
                                        robot.intake.raiseAdjustable,
                                        robot.intake.stop,
                                        robot.shooter.disable
                                )
                                .raceWith(waitMs(2000))
                ),
                Commands.infinite(() -> {
                            robot.update();
                            robot.shooter.enable();
                            telemetry.addData("pose", robot.pose());
                            telemetry.update();
                        }
                )
        );
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) {
            robot.follower.localizer.reset();
        }
    }

    public CommandBuilder setup(double velocity, double hood, double degrees) {
        return Commands.instant(() -> {
            robot.shooter.hood(hood);
            robot.shooter.target(velocity);
            robot.turret.angle(Math.toRadians(degrees));
        });
    }

    public void stop() {
        Tele.start = robot.pose();
        super.stop();
    }
}