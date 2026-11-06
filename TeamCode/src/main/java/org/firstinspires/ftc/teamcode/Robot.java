package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.utils.Alliance;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class Robot {
    public final Alliance alliance;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;
    public final Follower follower;
    private final Map<String, DcMotorEx> driveMotors = new LinkedHashMap<>();
    private final Timer loop = new Timer();
    public double loops = 0, lastLoop = 0, loopTime = 0;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        follower = Constants.create(hardwareMap);

        MecanumConfig drive = Constants.drivetrainConfig;
        for (String name : new String[]{drive.frontLeftName.get(), drive.frontRightName.get(), drive.backLeftName.get(), drive.backRightName.get()}) {
            driveMotors.put(name, hardwareMap.get(DcMotorEx.class, name));
        }

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
                waitUntil(shooter::atTarget)
                        .raceWith(waitMs(500)),
                intake.open,
                intake.transfer,
                waitMs(2000),
                intake.in,
                intake.close
        );
    }

    public CommandBuilder intake() {
        return intake.in.with(intake.close);
    }

    public double current() {
        return intake.current() + shooter.current() + followerCurrent();
    }

    public String currentBreakdown() {
        return intake.currentBreakdown() + "\n" + shooter.currentBreakdown() + "\n" + followerCurrentBreakdown();
    }

    public double followerCurrent() {
        double total = 0;
        for (DcMotorEx motor : driveMotors.values()) total += motor.getCurrent(CurrentUnit.AMPS);
        return total;
    }

    public String followerCurrentBreakdown() {
        StringBuilder sb = new StringBuilder();
        for (Map.Entry<String, DcMotorEx> e : driveMotors.entrySet()) {
            if (sb.length() > 0) sb.append("\n");
            sb.append(String.format(Locale.US, "%s: %.2fA", e.getKey(), e.getValue().getCurrent(CurrentUnit.AMPS)));
        }
        return sb.toString();
    }

    public Pose pose() {
        return follower.pose();
    }

    public void pose(Pose pose) {
        follower.setPose(pose);
    }
}
