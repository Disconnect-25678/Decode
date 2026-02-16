package org.firstinspires.ftc.teamcode.opmode.teleop;


import org.firstinspires.ftc.teamcode.subsystem.Superstructure;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "A-Tele", name = "BLUE")
public class BlueTeleOp extends TeleOp{
    public BlueTeleOp() {
        super(Superstructure.AllianceColor.BLUE);
    }
}
