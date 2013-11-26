import java.util.Stack;
import java.util.ArrayList;
import java.util.ListIterator;

import lejos.nxt.*;

public class Wrangler {

	Odometer odo;
	TwoWheeledRobot patbot;
	Detection detect;
	double finishX, finishY;
	double redX1, redY1;
	double redX2, redY2;
	double startX, startY;
	int widthX, widthY;
	Stack<Point> currentStack = new Stack<Point>();
	Stack<Point> backPedalStack = new Stack<Point>();
	Navigation navi;
	LightLocalizer localizer;
	UltraSensor ultra;
	NXTRegulatedMotor gateMotor;
	ArrayList<Point> dangerPointsL = new ArrayList<Point>();

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
		this.navi = new Navigation(this.odo/* , this.detect*/, this.ultra, this);
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

	/**
	 * Is the point Dangerous
	 * @param currentPoint
	 * @return
	 */
	private boolean isDangerous(Point currentPoint) {
		ListIterator<Point> iter = this.dangerPointsL.listIterator();
		Point nextPoint;
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

	/**
	 * 
	 */
	public void runSimpleCourse() {
		Point o = new Point();
		while (!this.currentStack.isEmpty()) {
			if (!isDangerous(this.currentStack.peek())
					&& !this.currentStack.peek().outOfBounds(this.widthX, this.widthY)) {
				this.currentStack.peek().setVisited();
				// navi move...
				this.currentStack = navi.travelTo(this.currentStack.peek().x*30.48, this.currentStack.peek().y*30.48, 15, this.currentStack, this.backPedalStack);
				//
				o = this.currentStack.peek();
				//if (!this.backPedalStack.isEmpty() && !Point.areSamePoints(this.currentStack.peek(), this.backPedalStack.peek())){
					this.backPedalStack.push(this.currentStack.pop());
				//}
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
