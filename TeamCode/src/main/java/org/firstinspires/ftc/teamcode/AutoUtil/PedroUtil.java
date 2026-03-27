package org.firstinspires.ftc.teamcode.AutoUtil;

import com.pedropathing.follower.Follower;

public class PedroUtil {
    public static boolean isFinished(Follower mp) {
        return ((mp.atParametricEnd() && mp.getHeadingError() < mp.getCurrentPath().getPathEndHeadingConstraint()) || !mp.isBusy());
    }
}
