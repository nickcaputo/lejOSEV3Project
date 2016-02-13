package main_threads;

import lejos.hardware.BrickFinder;
import lejos.hardware.lcd.GraphicsLCD;
/**
 * 
 * @author CGreen
 *
 */
public class MazeRevealMapTest {
	public static void main(String[] args) {
		GraphicsLCD screen = BrickFinder.getDefault().getGraphicsLCD();
		MazeRevealMap mazeMap = new MazeRevealMap(screen, 4, 4);
		System.out.println(mazeMap.drawWall(0, 0, 0));
		mazeMap.reset();
	}
}
