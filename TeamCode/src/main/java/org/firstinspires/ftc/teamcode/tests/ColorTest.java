package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorTest extends OpMode {

    RevColorSensorV3 colorl, colorr;
    MultipleTelemetry telemetryM;

    @Override
    public void init() {
        colorl = hardwareMap.get(RevColorSensorV3.class, "colorl");
        colorr = hardwareMap.get(RevColorSensorV3.class, "colorr");
        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        telemetryM.addData("distance l", colorl.getDistance(DistanceUnit.MM));
        telemetryM.addData("distance r", colorr.getDistance(DistanceUnit.MM));
        telemetryM.update();
    }
}
