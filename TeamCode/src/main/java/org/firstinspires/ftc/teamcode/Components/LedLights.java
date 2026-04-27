package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class LedLights {
    GoBildaPrismDriver prism;

    public LedLights(HardwareMap hwmap){
        prism = hwmap.get(GoBildaPrismDriver.class, "prism");
        prism.setStripLength(24);
    }

    public void blueColor(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Solid(Color.CYAN));
    }

    public void redColor(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Solid(new Color(255, 60, 0)));
    }

    public void greenColor(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Solid(Color.GREEN));
    }

    public void pinkColor(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Solid(Color.PINK));
    }

    public void sparkle(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, new PrismAnimations.Sparkle(Color.PINK, Color.CYAN));
    }

}
