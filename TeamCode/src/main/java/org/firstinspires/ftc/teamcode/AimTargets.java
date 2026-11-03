package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.utils.Alliance;

@Config
public class AimTargets {
    public final static Pose center = new Pose(70.75, 70.75);
    public static double xOffset = 13, yOffset = 10;

    public static Pose target(Alliance a, Pose currentPose) {
        Pose blueScoring = center.plus(new Pose(xOffset+12, yOffset));
        Pose redScoring = center.plus(new Pose(-xOffset-12, yOffset));
        Pose blueAudience = center.plus(new Pose(xOffset-12, -yOffset));
        Pose redAudience = center.plus(new Pose(-xOffset+12, -yOffset));

        if (a == Alliance.BLUE) {
            if (currentPose.y() > 70.75) {
                return blueScoring;
            } else {
                return blueAudience;
            }
        } else {
            if (currentPose.y() > 70.75) {
                return redScoring;
            } else {
                return redAudience;
            }
        }
    }
}
