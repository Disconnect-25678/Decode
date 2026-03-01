package org.firstinspires.ftc.teamcode.opmode.auto.leave;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystem.Superstructure;

@Autonomous(group = "Leave", name = "Red Leave (Goal Side)")
public class RedAutoLeave extends AutoLeave{
    public RedAutoLeave() {
        super(Superstructure.AllianceColor.RED);
    }
}
