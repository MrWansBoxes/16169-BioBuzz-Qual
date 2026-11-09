package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "Red Auto", preselectTeleOp = BioBuzzTeleOp.NAME)
public class RedAuto extends BioBuzzAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
