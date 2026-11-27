package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.utils.Alliance;

@Config
public class AimTargets {
    public static double blueScoringX = 84, blueScoringY = 86;
    public static double blueAudienceX = 84, blueAudienceY = 55.5;
    public static double redScoringX = 57.5, redScoringY = 86;
    public static double redAudienceX = 57.5, redAudienceY = 55.5;

    public static Pose target(Alliance a, Pose currentPose) {
        if (a == Alliance.BLUE) {
            if (currentPose.y() > 70.75) {
                return new Pose(blueScoringX, blueScoringY);
            } else {
                return new Pose(blueAudienceX, blueAudienceY);
            }
        } else {
            if (currentPose.y() > 70.75) {
                return new Pose(redScoringX, redScoringY);
            } else {
                return new Pose(redAudienceX, redAudienceY);
            }
        }
    }
}
