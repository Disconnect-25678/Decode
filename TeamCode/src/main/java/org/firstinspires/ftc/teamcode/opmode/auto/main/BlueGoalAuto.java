package org.firstinspires.ftc.teamcode.opmode.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystem.Superstructure;

@Autonomous(name = "Blue Goal Side", group = "Autos")
public class BlueGoalAuto extends AutoMain {
    public BlueGoalAuto() {
        super(AutoStartPosition.GOAL, Superstructure.AllianceColor.BLUE);
    }
}
