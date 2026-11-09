package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.prism.GoBildaPrismDriver;

public class Light {
    enum Colors {
        BLUE,
        RED,
        PURPLE,
        GREEN,
        ORANGE
    }

    private Colors current;
    GoBildaPrismDriver prism;
    public Light(HardwareMap hardwareMap) {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.enableDefaultBootArtboard(false);
    }

    public void in() {
        set(Colors.BLUE);
    }

    public void out() {
       set(Colors.PURPLE);
    }

    public void stop() {
        set(Colors.RED);
    }

    public void ready() {
        set(Colors.GREEN);
    }

    public void shoot() {
        set(Colors.ORANGE);
    }

    public void off() {
        prism.clearAllAnimations();
    }

    private void set(Colors color) {
        if (color.equals(current))
            return;

        current = color;

        switch (color) {
            case BLUE:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
                break;
            case GREEN:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
                break;
            case PURPLE:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
                break;
            case RED:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_3);
                break;
            case ORANGE:
                prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_4);
                break;
        }
    }
}
