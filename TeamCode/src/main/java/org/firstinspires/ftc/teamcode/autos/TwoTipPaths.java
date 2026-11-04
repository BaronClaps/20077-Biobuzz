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

    public final Pose start, flower1, flower1Control1, flower2, flower2Control1, flower2Control2, score, park;

    public TwoTipPaths(Follower follower, Alliance alliance) {
        this.follower = follower;

        if (alliance == Alliance.BLUE) {
            poseFactory = PoseFactory.degrees();
        } else {
            poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);
        }

        start = poseFactory.of(88, 134.5, 270);

        flower1 = poseFactory.of(92, 12.5, 270);
        flower1Control1 = poseFactory.of(72, 38, 0);

        flower2 = poseFactory.of(127, 94.33, 0);
        flower2Control1 = poseFactory.of(115, 40, 0);
        flower2Control2 = poseFactory.of(112, 96, 0);

        score = poseFactory.of(112, 11.5, 90);

        park = poseFactory.of(125, 24, 90);
    }

    public Pose start() {
        return start;
    }

    public CommandBuilder flower1() {
        return follow(follower,
                Paths.curve(start, flower1Control1, flower1)
                        .constant(flower1).with(Constants.foresightConfig.brakeAtEnd.at(false)));
    }

    public CommandBuilder flower1Shoot() {
        return follow(follower,
                Paths.line(flower1, score)
                        .linear(flower1, score));
    }

    public CommandBuilder flower2() {
        return follow(follower,
                Paths.curve(
                        score,
                        flower2Control1,
                        flower2Control2,
                        flower2
                ).linear(score, flower2)
                        .with(Constants.foresightConfig.brakeAtEnd.at(false)));
    }

    public CommandBuilder score() {
        return follow(follower,
                Paths.line(flower2, score)
                        .linear(flower2, score));
    }

    public CommandBuilder park() {
        return follow(follower,
                Paths.line(score, park)
                        .constant(park));
    }
}