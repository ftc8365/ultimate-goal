package org.firstinspires.ftc.teamcode.ultimategoal.robot;

import java.util.ArrayList;
import java.util.List;

public class Trajectory {
    List< Movement > movements = new ArrayList< Movement >();

    public Trajectory() {
    }

    List< Movement > getMovements() {
        return this.movements;
    }

}
