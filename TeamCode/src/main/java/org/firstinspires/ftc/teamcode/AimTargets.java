package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.utils.Alliance;

public class AimTargets {
    public static Pose blueScoring = new Pose(84, 80);
    public static Pose redScoring = new Pose(58, 80);
    public static Pose blueAudience = new Pose(84, 62);
    public static Pose redAudience = new Pose(58, 62);

    public static Pose target(Alliance a, Pose currentPose) {
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
