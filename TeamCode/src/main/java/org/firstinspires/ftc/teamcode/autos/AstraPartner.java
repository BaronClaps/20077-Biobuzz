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

public class AstraPartner extends CommandOpMode {
    final Alliance alliance;
    AstraPartnerPaths paths;
    Robot robot;

    public AstraPartner(Alliance alliance) {
        this.alliance = alliance;
    }

    public void init() {
        robot = new Robot(hardwareMap, alliance);
        paths = new AstraPartnerPaths(robot.follower, alliance);

        robot.pose(paths.start());
        robot.shooter.disable();
        robot.turret.angle(Math.toRadians(31.5));
        robot.intake.close();
        robot.intake.stop();
        robot.intake.lowerAdjustable();
        robot.intake.raiseFlower();

        schedule(
                sequential(
                        instant(() -> robot.pose(paths.start())),
                        robot.shooter.disable,
                        waitMs(5000),
                        robot.shooter.enable,
                        setup(1400, 0.275, 31.5),
                        waitUntil(() -> robot.shooter.atTarget()),
                        waitMs(500),
//                        waitMs(500),
                        robot.shoot(),
                        robot.shoot(),
                        paths.park()
                                .with(
                                        robot.intake.stop,
                                        robot.shooter.disable
                                )
                                .raceWith(waitMs(2000))
                ),
                Commands.infinite(() -> {
                            robot.update();
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
        Tele.start = paths.park;
        super.stop();
    }
}