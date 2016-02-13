package main_threads;

import lejos.hardware.BrickFinder;
import lejos.hardware.lcd.GraphicsLCD;

/**
 * Tests mapmaking by roving about a maze.
 * 
 * @author CGreen
 *
 */
public class Cartographer {
	public static void main(String[] args) {
		GraphicsLCD screen = BrickFinder.getDefault().getGraphicsLCD();
		GPS gps = new GPS(screen,0, 4, 0, 4, 4);
	}
}
