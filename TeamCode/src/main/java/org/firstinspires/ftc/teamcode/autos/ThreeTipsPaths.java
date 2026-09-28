package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;
import org.firstinspires.ftc.teamcode.utils.Alliance;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;

public class ThreeTipsPaths {
    public final PoseFactory poseFactory;
    public final Follower follower;

    public final Pose start, spikeIntake,
            spikeScore, spikeScoreControl1, spikeScoreSegment1Heading, spikeScoreSegment2Target,
            flower1Intake,
            flower2Intake, flower2IntakeControl1, flower2IntakeControl2, flower2IntakeSegment1Heading, flower2IntakeSegment2Target,
            sweepIntake, sweepIntakeControl1, sweepIntakeControl2, sweepIntakeSegment1Heading, sweepIntakeSegment2Start, sweepIntakeSegment2End,
            park, parkControl1, parkSegment2Heading;

    public ThreeTipsPaths(Follower follower, Alliance alliance){
        this.follower = follower;

        if (alliance == Alliance.BLUE) {
            poseFactory = PoseFactory.degrees();
        } else {
            poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);
        }

        start = poseFactory.of(120, 135.5, 90);
        spikeIntake = poseFactory.of(130, 135.5, 0);
        spikeScore = poseFactory.of(94, 20, -90);
        spikeScoreControl1 = poseFactory.of(110, 45, 0);
        spikeScoreSegment1Heading = poseFactory.of(94, 20, 0);
        spikeScoreSegment2Target = poseFactory.of(94, 0, 0);
        flower1Intake = poseFactory.of(94, 14, 270);
        flower2Intake = poseFactory.of(128, 94, 0);
        flower2IntakeControl1 = poseFactory.of(115, 35, 0);
        flower2IntakeControl2 = poseFactory.of(112, 100, 0);
        flower2IntakeSegment1Heading = poseFactory.of(128, 94, 270);
        flower2IntakeSegment2Target = poseFactory.of(141.5, 94, 0);
        sweepIntake = poseFactory.of(85, 110, -90.8208);
        sweepIntakeControl1 = poseFactory.of(125, 140, 0);
        sweepIntakeControl2 = poseFactory.of(85, 138, 0);
        sweepIntakeSegment1Heading = poseFactory.of(85, 110, 0);
        sweepIntakeSegment2Start = poseFactory.of(85, 110, 0);
        sweepIntakeSegment2End = poseFactory.of(85, 110, 180);
        park = poseFactory.of(130, 48, -90);
        parkControl1 = poseFactory.of(128, 110, 0);
        parkSegment2Heading = poseFactory.of(130, 48, 270);
    }

    public Pose start() {
        return start;
    }

    public CommandBuilder spikeIntake() {
        return follow(follower, Paths.line(start, spikeIntake).constant(spikeIntake));
    }

    public CommandBuilder spike1Score() {
        return follow(follower, Paths.curve(spikeIntake, spikeScoreControl1, spikeScore).heading(Interpolator.piecewise().until(0.1, Interpolator.constant(spikeScoreSegment1Heading)).until(1, Interpolator.facingPoint(spikeScoreSegment2Target))));
    }

    public CommandBuilder flower1Intake() {
        return follow(follower, Paths.line(spikeScore, flower1Intake).constant(flower1Intake));
    }

    public CommandBuilder flower2Intake() {
        return follow(follower, Paths.curve(flower1Intake, flower2IntakeControl1, flower2IntakeControl2, flower2Intake).heading(Interpolator.piecewise().until(0.1, Interpolator.constant(flower2IntakeSegment1Heading)).until(1, Interpolator.facingPoint(flower2IntakeSegment2Target))));
    }

    public CommandBuilder sweepIntake() {
        return follow(follower, Paths.curve(flower2Intake, sweepIntakeControl1, sweepIntakeControl2, sweepIntake).heading(Interpolator.piecewise().until(0.1, Interpolator.constant(sweepIntakeSegment1Heading)).until(0.3, Interpolator.linear(sweepIntakeSegment2Start, sweepIntakeSegment2End)).until(1, Interpolator.tangent)));
    }

    public CommandBuilder park() {
        return follow(follower, Paths.curve(sweepIntake, parkControl1, park).heading(Interpolator.piecewise().until(0.85, Interpolator.tangent).until(1, Interpolator.constant(parkSegment2Heading))));
    }
}