package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.Alliance;

public class AstraPartnerPaths {

    public final PoseFactory poseFactory;
    public final Follower follower;

    public final Pose start, park;

    public AstraPartnerPaths(Follower follower, Alliance alliance) {
        this.follower = follower;

        if (alliance == Alliance.RED) {
            poseFactory = PoseFactory.degrees();
        } else {
            poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);
        }

        start = poseFactory.of(32+6.25, 134.5, 270);

        park = poseFactory.of(7, 141.5-17.5, 270);
    }

    public Pose start() {
        return start;
    }

    public CommandBuilder park() {
        return follow(follower,
                Paths.line(start, park)
                        .constant(park));
    }
}