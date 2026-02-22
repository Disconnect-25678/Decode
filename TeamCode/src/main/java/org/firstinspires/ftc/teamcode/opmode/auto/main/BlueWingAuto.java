package org.firstinspires.ftc.teamcode.opmode.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystem.Superstructure;

@Autonomous(name = "Blue Wing Side", group = "Autos")
public class BlueWingAuto extends AutoMain {
    public BlueWingAuto() {
        super(AutoStartPosition.WING, Superstructure.AllianceColor.BLUE);
    }
}
