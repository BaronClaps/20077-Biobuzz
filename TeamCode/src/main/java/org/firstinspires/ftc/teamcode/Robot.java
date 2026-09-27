package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.utils.Alliance;

import java.util.List;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class Robot {
    public final Alliance alliance;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;
    public final Follower follower;
    private final Timer loop = new Timer();
    public double loops = 0, lastLoop = 0, loopTime = 0;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        follower = Constants.create(hardwareMap);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        loop.reset();

        update();
    }

    public void update() {
        loops++;

        if (loops > 10) {
            double now = loop.milliseconds();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        follower.update();
        shooter.update();
    }

    public CommandBuilder shoot() {
        return sequential(
                intake.openGate(),
                intake.shoot(),
                waitMs(750),
                intake.intake(),
                intake.closeGate()
        );
    }

    public CommandBuilder intake() {
        return intake.intake().with(intake.closeGate());
    }

    public Pose pose() {
        return follower.pose();
    }

    public void pose(Pose pose) {
        follower.setPose(pose);
    }
}
