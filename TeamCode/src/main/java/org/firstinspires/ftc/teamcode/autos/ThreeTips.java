package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CommandOpMode;

public class ThreeTips extends CommandOpMode {
    final Alliance alliance;
    ThreeTipsPaths paths;
    Robot robot;
    double velocity = 1560, hood = 0.25;

    public ThreeTips(Alliance alliance) {
        this.alliance = alliance;
    }

    public void init() {
        robot = new Robot(hardwareMap, alliance);
        paths = new ThreeTipsPaths(robot.follower, alliance);

        robot.pose(paths.start());
        robot.shooter.disable();
        robot.turret.angle(Math.toRadians(-120));
        robot.intake.close();
        robot.intake.stop();
        robot.intake.lowerAdjustable();
        robot.intake.raiseFlower();

        schedule(
                sequential(
                        setup(1540, 0.25, -118),
                        waitMs(2000),
                robot.shoot(),
                robot.shoot(),
                robot.intake(),
                        paths.spikeIntake(),
                        setup(1540, 0.25, -177.5),
                        paths.spike1Score(),
                        robot.intake.raiseAdjustable,
                        robot.intake.lowerFlower,
waitMs(1000),
                robot.shoot(),
                        robot.shoot(),
                        waitMs(500),
                robot.intake(),
                        waitMs(500),
                        paths.flower1Intake(),
                        waitMs(2000),
                        setup(1500, 0.25, -167),
                robot.shoot(),
robot.shoot()
//                robot.intake(),
//                        paths.flower2Intake(),
//                robot.shoot(),
//                robot.intake(),
//                        paths.sweepIntake(),
//                robot.shoot(),
//                robot.intake(),
//                        paths.park()
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
