import java.util.Stack;
import java.util.ArrayList;
import java.util.ListIterator;

import lejos.nxt.*;

public class Wrangler {

	private Odometer odo;
	private TwoWheeledRobot patbot;
	private Detection detect;
	private double finishX, finishY;
	private double redX1, redY1;
	private double redX2, redY2;
	private double greenX1, greenY1;
	private double greenX2, greenY2;
	private double greenMidX, greenMidY;
	private double startX, startY;
	private int widthX, widthY;
	private Stack<Point> currentStack = new Stack<Point>();
	public Stack<Point> backPedalStack = new Stack<Point>();
	private Navigation navi;
	private LightLocalizer localizer;
	private UltraSensor ultra;
	private NXTRegulatedMotor gateMotor;
	private ArrayList<Point> dangerPointsL = new ArrayList<Point>();

	// Point dangerPoints[] = new Point[12];

	/**
	 * 
	 * @param odo
	 * @param patbot
	 * @param detect
	 * @param ultra
	 * @param gateMotor
	 */
	public Wrangler(Odometer odo, TwoWheeledRobot patbot,
			Detection detect, // LightLocalizer localizer,
			UltraSensor ultra, NXTRegulatedMotor gateMotor) {
		this.odo = odo;
		this.patbot = patbot;
		this.detect = detect;
		this.ultra = ultra;
		this.navi = new Navigation(this.odo, this.ultra, this, detect);
		// this.localizer = localizer;
	}

	/**
	 * Setter for Start Position
	 * @param startX
	 * @param startY
	 */
	public void setStarter(double startX, double startY) {
		this.startX = startX;
		this.startY = startY;
	}

	/**
	 * Setter for Final Position
	 * @param finishX
	 * @param finishY
	 */
	public void setFinal(double finishX, double finishY) {
		this.finishX = finishX;
		this.finishY = finishY;
	}

	/**
	 * Setter for ArenaSize
	 * @param widthX
	 * @param widthY
	 */
	public void setArenaSize(int widthX, int widthY) {
		this.widthX = widthX;
		this.widthY = widthY;
	}
	
	/**
	 * Sets original RedZone
	 * @param redX1
	 * @param redY1
	 * @param redX2
	 * @param redY2
	 */
	public void setRedZone(double redX1, double redY1, double redX2, double redY2){
		this.redX1 = redX1;
		this.redY1 = redY1;
		this.redX2 = redX2;
		this.redY2 = redY2;		
	}

	/** Sets finalGreenZone;
	 * 
	 * @param greenX1
	 * @param greenY1
	 * @param greenX2
	 * @param greenY2
	 */
	public void setGreenZone(double greenX1, double greenY1, double greenX2, double greenY2){
		this.greenX1 = greenX1;
		this.greenY1 = greenY1;
		this.greenX2 = greenX2;
		this.greenY2 = greenY2;
	}
	
	/** Sets middle of GreenZone for finalPosition
	 * 
	 */
	public void setGreenMid(){
		this.finishX = (this.greenX1 + this.greenX2)/2;
		this.finishY = (this.greenY1 + this.greenY2)/2;
	}
	
	/** Adds the GreenZone to the RedZone for Avoidance
	 * 
	 */
	public void addGreenZoneToRedZone(){
		int stepsX = (int) Math.round(Math.abs(this.greenX2 - this.greenX1));
		int stepsY = (int) Math.round(Math.abs(this.greenY2 - this.greenY1));
		for (int i = 0; i <= stepsX; i++) {
			for (int j = 0; j <= stepsY; j++) {
				this.dangerPointsL.add(new Point((greenX1 + i), (greenY1 + j), false));
			}
		}
	}
	
	/** returns finish X
	 * 
	 * @return
	 */
	public double getFinishX(){
		return this.finishX;
	}
	
	/** returns Finish Y
	 * 
	 * @return
	 */
	public double getFinishY(){
		return this.finishY;
	}
	
	/** Inserts corners into Red Zone
	 * 
	 * @param startID
	 */
	public void insertCornersIntoRedZone(int startID){
		if (startID != 1) {
			insertIntoDangerList(0, 0, 0, 0);
		}
		if (startID != 2) {
			insertIntoDangerList(this.widthX, 0, this.widthX, 0);
		}
		if (startID != 3) {
			insertIntoDangerList(this.widthX, this.widthY, this.widthX, this.widthY);
		}
		if (startID != 4) {
			insertIntoDangerList(0, this.widthY, 0, this.widthY);
		}
	}
	
	/** Removes GreenZone from the RedZone() once everything is done
	 * 
	 */
	public void removeGreenZoneFromRedZone() {
		int stepsX = (int) Math.round(Math.abs(this.greenX2 - this.greenX1));
		int stepsY = (int) Math.round(Math.abs(this.greenY2 - this.greenY1));
		for (int i = 0; i <= stepsX; i++) {
			for (int j = 0; j <= stepsY; j++) {
				this.dangerPointsL.remove(new Point((greenX1 + i), (greenY1 + j),
						false));
			}
		}
	}

	/**
	 * Creates the initial DangerList
	 */
	public void createDangerList() {
		int stepsX = (int) Math.round(Math.abs(this.redX2 - this.redX1));
		int stepsY = (int) Math.round(Math.abs(this.redY2 - this.redY1));
		for (int i = 0; i <= stepsX; i++) {
			for (int j = 0; j <= stepsY; j++) {
				this.dangerPointsL.add(new Point((redX1 + i), (redY1 + j), false));
			}
		}
	}
	
	/**
	 * Inserts new RedZone into DangerList
	 * @param redX1
	 * @param redY1
	 * @param redX2
	 * @param redY2
	 */
	public void insertIntoDangerList(double redX1, double redY1,
			double redX2, double redY2) {
		int stepsX = (int) Math.round(Math.abs(redX2 - redX1));
		int stepsY = (int) Math.round(Math.abs(redY2 - redY1));
		for (int i = 0; i <= stepsX; i++) {
			for (int j = 0; j <= stepsY; j++) {
				this.dangerPointsL.add(new Point((redX1 + i), (redY1 + j), false));
			}
		}
	}
	/** Returns true if robot is inside the greenzone
	 * @param x
	 * @param y 
	 */
	public boolean insideGreenZone(double x, double y){
		if (x > this.greenX1*30.48 && x < this.greenX2*30.48 && y> this.greenY1*30.48 && y < this.greenY2*30.48){
			return true;
		} else if (Math.sqrt(Math.pow((x - this.finishX*30.48), 2) + Math.pow((y - this.finishY*30.48), 2)) <= 100.0){
			return true;
		}
		else {
			return false;
		}
	}
	
	/**
	 * Is the point Dangerous
	 * @param currentPoint
	 * @return
	 */
	private boolean isDangerous(Point currentPoint) {
		ListIterator<Point> iter = this.dangerPointsL.listIterator();
		Point nextPoint;
		//Iterates through dangerList until it finds that specific point to return true or false
		while (iter.hasNext()) {
			nextPoint = iter.next();
			if (Point.areSamePoints(currentPoint, nextPoint)) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Generates the initial Set Path
	 */
	public void generateSetPath() {
		double currentX = this.startX;
		double currentY = this.startY;
		double tempY = this.widthY;
		int i;
		if (currentX < (this.widthX) / 2) {
			i = 0;
		} else {
			i = 1;
		}
		// pushes points in reverse order from the opposite corner into the robot in an s pattern 
		// while ignoring redzones
		while (tempY >= 0) {
			if (i % 2 == 0) {
				for (int j = (int) this.widthX; j >= 0; j--) {
					this.currentStack.push(new Point((j), Math.abs(currentY
							- tempY), false));
				}
			} else {
				for (int j = 0; j <= (int) this.widthX; j++) {
					this.currentStack.push(new Point((j), Math.abs(currentY
							- tempY), false));
				}
			}
			tempY--;
			i++;

		}
	}
	
	/**
	 * Generates a non-Set Path
	 * @param finish
	 * @param startPointX
	 * @param startPointY
	 * @param endPointX
	 * @param endPointY
	 * @param firstStack
	 * @return
	 */
	public Stack<Point> generatePath(boolean finish, double startPointX,
			double startPointY, double endPointX, double endPointY, Stack<Point> firstStack) {
		Stack<Point> generatedStack = firstStack;
		Stack<Point> backpedalStack = new Stack<Point>();
		double currentX = startPointX;
		double currentY = startPointY;
		double displacementX = (endPointX - currentX);
		double displacementY = (endPointY - currentY);
		Point endPoint = new Point(endPointX, endPointY, false);
		//Similar to generatePath except it refuses to perform path generation if the point is dangerous
		// Creates a step-ladder path to the finish point
		if (!isDangerous(endPoint)) {
			boolean doneX = false, doneY = false;
			while (!doneX && !doneY) {
				if (!doneX) {
					Point nextP = new Point((currentX + displacementX / Math.abs(displacementX)), currentY, false);
					if (!isDangerous(nextP)) {
						if (endPointX != (currentX)) {
							currentX += displacementX / Math.abs(displacementX);
							if (finish) {
								generatedStack.push(nextP);
							}
							backpedalStack.push(nextP);
						} else {
							doneX = true;
						}
					}
				}
				if (!doneY) {
					Point nextP = new Point(currentX, currentY + displacementY / Math.abs(displacementY), false);
					if (!isDangerous(nextP)) {
						if (endPointY != currentY) {
							currentY += displacementY / Math.abs(displacementY);
							if (finish) {
								generatedStack.push(nextP);
							}
							backpedalStack.push(nextP);
						} else {
							doneY = true;
						}
					}
				}
			}
			while (!backpedalStack.empty()) {
				if (generatedStack.isEmpty() || !Point.areSamePoints(backpedalStack.peek(), generatedStack.peek())) {
					generatedStack.push(backpedalStack.pop());
				} else {
					backpedalStack.pop();
				}
			}
		}

		return generatedStack;
	}

	/** Runs the course according to a stack
	 * 
	 */
	public void runSimpleCourse() {
		
		Point o = new Point();
		while (!this.currentStack.isEmpty()) {
			if (!isDangerous(this.currentStack.peek())
					&& !this.currentStack.peek().outOfBounds(this.widthX, this.widthY)) {
				LCD.clear(5);
				LCD.drawString(currentStack.peek().pointToString(), 0, 5);
				this.currentStack.peek().setVisited();
				// navi move...
				this.currentStack = navi.travelTo(false, this.currentStack.peek().x*30.48, this.currentStack.peek().y*30.48, 15, this.currentStack, this.backPedalStack);
				o = this.currentStack.peek();
				LCD.clear(6);
				if (!backPedalStack.isEmpty()){
					LCD.drawString(backPedalStack.peek().pointToString(), 0, 6);
				}
				this.backPedalStack.push(this.currentStack.pop());
			
			} else {
				this.currentStack.pop();
				if (!currentStack.isEmpty()){
					Point nextP = this.currentStack.peek();
					if (isDangerous(Point.upAdjacentPoint(this.backPedalStack.peek()))
							|| isDangerous(Point.downAdjacentPoint(this.backPedalStack.peek()))) {
						if (this.backPedalStack.peek().x <= (this.redX1 + this.redX2) / 2) {
							this.currentStack.push(Point.leftAdjacentPoint(this.backPedalStack.peek()));
							this.backPedalStack.push(Point.leftAdjacentPoint(this.backPedalStack.peek()));
						} else {
							this.currentStack.push(Point.rightAdjacentPoint(this.backPedalStack.peek()));
							this.backPedalStack.push(Point.rightAdjacentPoint(this.backPedalStack.peek()));
						}
					} else if (isDangerous(Point.rightAdjacentPoint(this.backPedalStack.peek()))
							|| isDangerous(Point.leftAdjacentPoint(this.backPedalStack.peek()))) {
						if (this.backPedalStack.peek().y < (this.redY1 + this.redY2) / 2) {
							this.currentStack.push(Point.downAdjacentPoint(this.backPedalStack.peek()));
							backPedalStack.push(Point.downAdjacentPoint(this.backPedalStack.peek()));
						} else {
							this.currentStack.push(Point.upAdjacentPoint(this.backPedalStack.peek()));
							this.backPedalStack.push(Point.upAdjacentPoint(this.backPedalStack.peek()));
						}
					}
					this.currentStack = generatePath(false, o.x, o.y, nextP.x, nextP.y,
							this.currentStack);
				}
			}
		}
	}	

	/**
	 * 
	 * @return
	 */
	public boolean isTurning(){
		return navi.isTurning();
	}

	/**
	 * 
	 * @return
	 */
	public Navigation getNav(){
		return this.navi;
	}
}
