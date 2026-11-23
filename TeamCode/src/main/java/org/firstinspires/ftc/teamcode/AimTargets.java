package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.utils.Alliance;

@Config
public class AimTargets {
    public static final Pose blueScoring = new Pose(84, 86);
    public static final Pose blueAudience = new Pose(84, 55.5);
    public static final Pose redScoring = new Pose(57.5, 86);
    public static final Pose redAudience = new Pose(57.5, 55.5);

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
