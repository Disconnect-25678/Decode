package org.firstinspires.ftc.teamcode.opmode.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystem.Superstructure;

@Autonomous(name = "Red Goal Side", group = "Autos")
public class RedGoalAuto extends AutoMain {
    public RedGoalAuto() {
        super(AutoStartPosition.GOAL, Superstructure.AllianceColor.RED);
    }
}
