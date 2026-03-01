package org.firstinspires.ftc.teamcode.opmode.auto.leave;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystem.Superstructure;

@Autonomous(group = "Leave", name = "Blue Leave (Goal Side)")
public class BlueAutoLeave extends AutoLeave{
    public BlueAutoLeave() {
        super(Superstructure.AllianceColor.BLUE);
    }
}
