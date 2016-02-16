package main_threads;

import lejos.hardware.lcd.GraphicsLCD;

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
		//screenWidth = lcdScreen.getWidth(); // make the map rectangular (fill the screen)
		screenWidth = screenHeight; // make the map a square
		this.numRows = numRows;
		this.numColumns = numColumns;
		cellWidth = screenWidth/numColumns;
		cellHeight = screenHeight/numRows;
	}
	
	/**
	 * Displays a blank map.
	 */
	public void reset() {
		lcdScreen.clear();
		lcdScreen.drawRect(0, 0, screenWidth-1, screenHeight-1); 
			/* subtracting one keeps it from going off the screen*/
		
		// Draw dashed lines to show cell positions (still deciding whether to do this)
		
		/*lcdScreen.setStrokeStyle(1);
		for (int i = 0; i < numColumns; i++) {
			for (int j = 0; j < numRows; j++) {
				lcdScreen.drawRect(cellWidth*i, cellHeight*j, cellWidth, cellHeight);
			}
		}*/
		
	}
	
	/**
	 * Draws a wall on the map beside the robot in a particular direction.
	 * 
	 * @param x - x-coordinate of the cell
	 * @param y - y-coordinate of the cell
	 * @param direction - cardinal direction of the side of the cell where there is a wall
	 * @return - success/failure of the operation
	 */
	public boolean drawWall(int x, int y, int direction) {
		//TODO: Should fail if given coordinates are outside the map.
		lcdScreen.setStrokeStyle(0);
		
		switch (direction) {
		case 0: // suppose north wall
			lcdScreen.drawLine(cellWidth * x, cellHeight * (numRows - 1 - y), cellWidth * (x + 1), cellHeight * (numRows - 1 - y));
			break;
		case 1: // suppose east wall
			lcdScreen.drawLine(cellWidth * (x + 1), cellHeight * (numRows - 1 - y), cellWidth * (x + 1), cellHeight * ((numRows - 1 - y) + 1));
			break;
		case 2:// suppose south wall
			lcdScreen.drawLine(cellWidth * x, cellHeight * ((numRows - 1 - y) + 1), cellWidth * (x + 1), cellHeight * ((numRows - 1 - y) + 1));
			break;
		case 3:// suppose west wall
			lcdScreen.drawLine(cellWidth * x, cellHeight * (numRows - 1 - y), cellWidth * x, cellHeight * ((numRows - 1 - y) + 1));
			break;
		}
		return true;
	}
	
}
