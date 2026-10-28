package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

public class TwoTipPaths {

    public final PoseFactory poseFactory;
    public final Follower follower;

    public final Pose start, flower1, flower1Control1, flower2Start, flower2, flower2Control1, flower2Control2, scoreStart, score, park, parkSegment2Heading;

    public TwoTipPaths(Follower follower, Alliance alliance) {
        this.follower = follower;

        if (alliance == Alliance.BLUE) {
            poseFactory = PoseFactory.degrees();
        } else {
            poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);
        }

        start = poseFactory.of(88, 134.5, 270);

        flower1 = poseFactory.of(92, 13, 270);
        flower1Control1 = poseFactory.of(72, 38, 0);

        flower2Start = poseFactory.of(94.33, 14, 270);
        flower2 = poseFactory.of(127, 94.33, 0);
        flower2Control1 = poseFactory.of(115, 40, 0);
        flower2Control2 = poseFactory.of(112, 96, 0);

        scoreStart = poseFactory.of(125, 94.33, 0);
        score = poseFactory.of(120, 11.5, 45);

        park = poseFactory.of(125, 45, -90);
        parkSegment2Heading = poseFactory.of(125, 45, -90);
    }

    public Pose start() {
        return start;
    }

    public CommandBuilder flower1() {
        return follow(follower,
                Paths.curve(start, flower1Control1, flower1)
                        .constant(flower1).with(Constants.foresightConfig.brakeAtEnd.at(false)));
    }

    public CommandBuilder flower2() {
        return follow(follower,
                Paths.curve(
                        flower2Start,
                        flower2Control1,
                        flower2Control2,
                        flower2
                ).linear(flower2Start, flower2)
                        .with(Constants.foresightConfig.brakeAtEnd.at(false)));
    }

    public CommandBuilder score() {
        return follow(follower,
                Paths.line(scoreStart, score)
                        .linear(scoreStart, score));
    }

    public CommandBuilder park() {
        return follow(follower,
                Paths.line(score, park)
                        .heading(
                                Interpolator.piecewise()
                                        .until(0.8, Interpolator.tangent)
                                        .until(1, Interpolator.constant(parkSegment2Heading))
                        ));
    }
}