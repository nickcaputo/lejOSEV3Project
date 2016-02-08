package main_threads;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.utility.Delay;

public class HelloWorld
{
    public static void main(String[] args)
    {
        GraphicsLCD lcd = BrickFinder.getDefault().getGraphicsLCD();
        final int SW = lcd.getWidth();
        final int SH = lcd.getHeight();
        Button.LEDPattern(4);
        Sound.beepSequenceUp();
        
        lcd.setFont(Font.getSmallFont());
        lcd.drawString("My name is DaveKillbot!", SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        Button.LEDPattern(3);
        Delay.msDelay(4000);
        Button.LEDPattern(5);
        lcd.clear();
        lcd.refresh();
        Sound.beepSequence();
        Delay.msDelay(500);
        Button.LEDPattern(0);
    }
}
