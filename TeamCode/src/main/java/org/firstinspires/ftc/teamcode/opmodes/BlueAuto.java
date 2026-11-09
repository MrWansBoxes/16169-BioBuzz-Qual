package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "Blue Auto", preselectTeleOp = BioBuzzTeleOp.NAME)
public class BlueAuto extends BioBuzzAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
