package main_threads;

import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.utility.Delay;

public class MazeRevealMap {
	private GraphicsLCD lcdScreen;
	private int screenHeight, screenWidth, cellWidth, cellHeight, numRows, numColumns;
	
	
	
	public MazeRevealMap(GraphicsLCD lcdScreen, int numRows, int numColumns) {
		this.lcdScreen = lcdScreen;
		screenHeight = lcdScreen.getHeight();
		screenWidth = lcdScreen.getWidth();
		// cell width and height only good for current test
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
	 * Displays an blank map. (For testing purposes, this method currently
	 * displays a blank map, then a map with all interior walls).
	 */
	public void reset() {
		lcdScreen.clear();
		lcdScreen.setStrokeStyle(0);
		lcdScreen.drawRect(0, 0, screenWidth-1, screenHeight-1);
		Delay.msDelay(5000);
		/* Create a grid of cells. I think it would be neat to draw one wall
		 * at a time; like, when Dave scans his surroundings whilst sitting in a
		 * cell, he fills in walls on the map where they are detected. At the
		 * moment, to test the look of the map, it just draws rectangles for
		 * each cell.
		 */
		for (int row = 0; row < numRows; row++) {
			for (int column = 0; column < numColumns; column++) {
				lcdScreen.drawRect(cellWidth*column, cellHeight*row, cellWidth, cellHeight);
			}
		}
		Delay.msDelay(5000);
		/*Sound.playTone(587, 2*63);
		Sound.playTone(587, 2*126);
		Sound.playTone(587, 2*126);
		Sound.playTone(466, 2*63);
		Sound.playTone(587, 2*126);
		Sound.playTone(698, 2*252);
		Sound.playTone(349, 2*252);*/
	}
	
}
