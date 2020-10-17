package org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling;

import java.util.ArrayList;
import java.util.List;

public class Trajectory {
    List<Motion> motions = new ArrayList< Motion >();

    public Trajectory() {
    }

    public List< Motion > getMotions() {
        return this.motions;
    }
}
