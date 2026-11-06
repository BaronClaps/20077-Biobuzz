package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.Robot;
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
        robot.turret.angle(Math.toRadians(-105));
        robot.intake.close();
        robot.intake.stop();
        robot.intake.lowerAdjustable();
        robot.intake.raiseFlower();

        schedule(
                sequential(
                        setup(1440, 0.26, -110),
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
                        setup(1440, 0.26, -110),
                        robot.intake.stop,
                        robot.shoot(),
                        robot.intake.raiseAdjustable,
                        robot.intake(),
                        paths.flower1()
                                .with(
                                        robot.intake.stop,
                                        robot.intake.lowerAdjustable,
                                        waitUntil(() -> robot.follower.completion() > 0.7)
                                                .then(
                                                        robot.intake.raiseAdjustable,
                                                        robot.intake.lowerFlower
                                                ),
                                        waitUntil(() -> robot.follower.completion() > 0.8)
                                                .then(
                                                        robot.intake.in
                                                )
                                )
                                .raceWith(waitMs(5000)),
                        setup(1440, 0.26, 35),
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
                        setup(1440, 0.26, 35),
                        paths.score()
                                .with(
                                        waitUntil(() -> robot.follower.completion() > 0.2)
                                                .then(
                                                        robot.intake.raiseAdjustable
                                                )
                                )
                                .raceWith(waitMs(5000)),
                        robot.intake.lowerAdjustable,
                        waitMs(500),
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
                        }
                )
        );
    }

    public CommandBuilder setup(double velocity, double hood, double degrees) {
        return Commands.instant(() -> {
            robot.shooter.hood(hood);
            robot.shooter.target(velocity);
            robot.turret.angle(Math.toRadians(degrees));
        });
    }
}