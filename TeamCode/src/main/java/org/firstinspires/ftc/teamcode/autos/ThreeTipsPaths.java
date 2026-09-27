package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import org.firstinspires.ftc.teamcode.utils.Alliance;

public class ThreeTipsPaths {
    public final PoseFactory poseFactory;
    public final Pose start, spikeIntake,
            spike1Score, spike1ScoreControl1, spike1ScoreSegment1Heading, spike1ScoreSegment2Target,
            flower1Intake,
            flower2Intake, flower2IntakeControl1, flower2IntakeControl2, flower2IntakeSegment1Heading, flower2IntakeSegment2Target,
            sweepIntake, sweepIntakeControl1, sweepIntakeControl2, sweepIntakeSegment1Heading, sweepIntakeSegment2Start, sweepIntakeSegment2End,
            park, parkControl1, parkSegment2Heading;

    public ThreeTipsPaths(Alliance alliance){
        
        if (alliance == Alliance.BLUE) {
            poseFactory = PoseFactory.degrees();
        } else {
            poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);
        }

        start = poseFactory.of(120, 135.5, 90);
        spikeIntake = poseFactory.of(130, 135.5, 0);
        spike1Score = poseFactory.of(94, 20, -90);
        spike1ScoreControl1 = poseFactory.of(110, 45, 0);
        spike1ScoreSegment1Heading = poseFactory.of(94, 20, 0);
        spike1ScoreSegment2Target = poseFactory.of(94, 0, 0);
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

    public Path spikeIntake() {
        return Paths.line(start, spikeIntake).constant(spikeIntake);
    }

    public Path spike1Score() {
        return Paths.curve(spikeIntake, spike1ScoreControl1, spike1Score).heading(Interpolator.piecewise().until(0.1, Interpolator.constant(spike1ScoreSegment1Heading)).until(1, Interpolator.facingPoint(spike1ScoreSegment2Target)));
    }

    public Path flower1Intake() {
        return Paths.line(spike1Score, flower1Intake).constant(flower1Intake);
    }

    public Path flower2Intake() {
        return Paths.curve(flower1Intake, flower2IntakeControl1, flower2IntakeControl2, flower2Intake).heading(Interpolator.piecewise().until(0.1, Interpolator.constant(flower2IntakeSegment1Heading)).until(1, Interpolator.facingPoint(flower2IntakeSegment2Target)));
    }

    public Path sweepIntake() {
        return Paths.curve(flower2Intake, sweepIntakeControl1, sweepIntakeControl2, sweepIntake).heading(Interpolator.piecewise().until(0.1, Interpolator.constant(sweepIntakeSegment1Heading)).until(0.3, Interpolator.linear(sweepIntakeSegment2Start, sweepIntakeSegment2End)).until(1, Interpolator.tangent));
    }

    public Path park() {
        return Paths.curve(sweepIntake, parkControl1, park).heading(Interpolator.piecewise().until(0.85, Interpolator.tangent).until(1, Interpolator.constant(parkSegment2Heading)));
    }
}