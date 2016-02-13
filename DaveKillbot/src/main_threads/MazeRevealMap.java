package main_threads;

import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.utility.Delay;
/**
 * 
 * @author CGreen
 *
 */
public class MazeRevealMap {
	private GraphicsLCD lcdScreen;
	private int screenHeight, screenWidth, cellWidth, cellHeight, numRows, numColumns;
	
	public MazeRevealMap(GraphicsLCD lcdScreen, int numRows, int numColumns) {
		this.lcdScreen = lcdScreen;
		lcdScreen.setStrokeStyle(0);
		screenHeight = lcdScreen.getHeight();
		screenWidth = lcdScreen.getWidth();
		this.numRows = numRows;
		this.numColumns = numColumns;
		cellWidth = screenWidth/numColumns;
		cellHeight = screenHeight/numRows;
	}
	/**
	 * Refreshes the screen display with the current map data.
	 */
	public void update() {
		
	}
	/**
	 * Displays a blank map.
	 */
	public void reset() {
		lcdScreen.clear();
		lcdScreen.drawRect(0, 0, screenWidth-1, screenHeight-1); /* subtracting one
													keeps it from going off the screen*/
		
		/*Sound.playTone(587, 2*63);
		Sound.playTone(587, 2*126);
		Sound.playTone(587, 2*126);
		Sound.playTone(466, 2*63);
		Sound.playTone(587, 2*126);
		Sound.playTone(698, 2*252);
		Sound.playTone(349, 2*252);*/
	}
	/**
	 * Draws a wall on the map in front of the robot's view.
	 * 
	 * @param x
	 * @param y
	 * @param direction
	 */
	public boolean drawWall(int x, int y, int direction) {
		// prevent walls from being drawn on outermost maze boundaries
		if (x == 0 && direction == 3 || x == numColumns - 1 && direction == 1
				|| y == 0 && direction == 0 || y == numRows - 1 && direction == 2
				|| lcdScreen == null) {
			return false;
		} else {
			switch (direction) {
			case 0: // suppose north wall
				lcdScreen.drawLine(cellWidth * x, cellHeight * y, cellWidth * (x + 1), cellHeight * y);
				break;
			case 1: // suppose east wall
				lcdScreen.drawLine(cellWidth * (x + 1), cellHeight * y, cellWidth * (x + 1), cellHeight * (y + 1));
				break;
			case 2:// suppose south wall
				lcdScreen.drawLine(cellWidth * x, cellHeight * (y + 1), cellWidth * (x + 1), cellHeight * (y + 1));
				break;
			case 3:// suppose west wall
				lcdScreen.drawLine(cellWidth * x, cellHeight * y, cellWidth * x, cellHeight * (y + 1));
				break;
			}
			return true;
		}

	}
	
}
