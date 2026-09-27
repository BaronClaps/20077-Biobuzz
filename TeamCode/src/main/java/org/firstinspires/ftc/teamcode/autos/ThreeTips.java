package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CommandOpMode;

public class ThreeTips extends CommandOpMode {
    final Alliance alliance;
    ThreeTipsPaths paths;
    Robot robot;

    public ThreeTips(Alliance alliance) {
        this.alliance = alliance;
    }

    public void init() {
        robot = new Robot(hardwareMap, alliance);
        paths = new ThreeTipsPaths(robot.follower, alliance);

        robot.pose(paths.start());
        robot.shooter.disable();
        robot.turret.angle(0);
        robot.intake.close();
        robot.intake.stop();

        schedule(
                robot.shoot(),
                robot.intake(),
                paths.spikeIntake(),
                robot.shoot(),
                robot.intake(),
                paths.spike1Score(),
                robot.shoot(),
                robot.intake(),
                paths.flower1Intake(),
                robot.shoot(),
                robot.intake(),
                paths.flower2Intake(),
                robot.shoot(),
                robot.intake(),
                paths.sweepIntake(),
                robot.shoot(),
                robot.intake(),
                paths.park()
        );
    }
}
