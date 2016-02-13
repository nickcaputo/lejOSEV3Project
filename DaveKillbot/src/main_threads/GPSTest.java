package main_threads;

import lejos.hardware.BrickFinder;
import lejos.hardware.lcd.GraphicsLCD;
/**
 * 
 * @author CGreen
 *
 */
public class GPSTest {
	public static void main(String[] args) {
		GraphicsLCD screen = BrickFinder.getDefault().getGraphicsLCD();
		GPS gps = new GPS(screen,0,3,0,4,4);
		
	}
}
