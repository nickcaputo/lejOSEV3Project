package main_threads;

import lejos.hardware.BrickFinder;
import lejos.hardware.lcd.GraphicsLCD;

public class GPSTest {
	public static void main(String[] args) {
		GraphicsLCD screen = BrickFinder.getDefault().getGraphicsLCD();
		GPS gps = new GPS(0,0,0,4,4, screen);
		
	}
}
