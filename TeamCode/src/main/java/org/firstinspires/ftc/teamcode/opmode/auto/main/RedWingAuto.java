package org.firstinspires.ftc.teamcode.opmode.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystem.Superstructure;

@Autonomous(name = "Red Wing Side", group = "Autos")
public class RedWingAuto extends AutoMain {
    public RedWingAuto() {
        super(AutoStartPosition.WING, Superstructure.AllianceColor.RED);
    }
}
