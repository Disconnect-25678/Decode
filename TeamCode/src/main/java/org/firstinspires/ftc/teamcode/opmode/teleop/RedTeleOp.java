package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.teamcode.subsystem.Superstructure;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "A-Tele")
public class RedTeleOp extends TeleOp{
    public RedTeleOp() {
        super(Superstructure.AllianceColor.RED);
    }
}
