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
        robot.intake.raiseAdjustable();
        robot.intake.raiseFlower();

        schedule(
                sequential(
                        setup(1560, 0.25, -120),
//                robot.shoot(),
//                robot.intake(),
                        paths.spikeIntake(),
//                robot.shoot(),
//                robot.intake(),
                        setup(1560, 0.35, -165),
                        paths.spike1Score(),
                        robot.intake.lowerFlower,
//                robot.shoot(),
//                robot.intake(),
                        setup(1560, 0.35, -165),
                        paths.flower1Intake(),
                        waitMs(2000)
//                robot.shoot(),
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
        return Commands.infinite(() -> {
            robot.shooter.hood(hood);
            robot.shooter.target(velocity);
            robot.turret.angle(Math.toRadians(degrees));
        });
    }
}
