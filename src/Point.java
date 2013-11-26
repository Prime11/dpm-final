public class Point {
	double x;
	double y;
	boolean visited;

	/**
	 * Simple 0,0 Constructor
	 */
	public Point(){
		this.x = 0;
		this.y = 0;
		this.visited = false;
	}

	/**
	 * Constructor
	 * @param x
	 * @param y
	 * @param visited
	 */
	public Point(double x, double y, boolean visited) {
		this.x = x;
		this.y = y;
		this.visited = visited;
	}

	/**
	 * Sets Point to visited
	 */
	public void setVisited() {
		this.visited = true;
	}

	/**
	 * Returns if point is outOfBounds
	 * @param widthX
	 * @param widthY
	 * @return
	 */
	public boolean outOfBounds(double widthX, double widthY){
		return this.x < 0 || this.x > widthX || this.y < 0 || this.y > widthY;
	}

	/**
	 * Returns Point to String
	 * @return
	 */
	public String pointToString() {
		return "[" + this.x + ", " + this.y + ", " + this.visited + "]";
	}

	/**
	 * Returns Point to Coords if Point is in Distance
	 * @param stepCase
	 * @return
	 */
	public String pointToCoords(double stepCase) {
		return "[" + this.x / stepCase + ", " + this.y / stepCase + ", "
				+ this.visited + "]";
	}

	/**
	 * Returns point to Tabbed Version
	 * @return
	 */
	public String pointToTV() {
		return "" + this.x + "     " + this.y;
	}

	/**
	 * Returns true if points are the same
	 * @param A
	 * @param B
	 * @return
	 */
	public static boolean areSamePoints(Point A, Point B) {
		return (Double.compare(A.x, B.x) == 0)
				&& (Double.compare(A.y, B.y) == 0) && (A.visited == B.visited);
	}

	/**
	 * Returns Left Adjacent Point
	 * @param A
	 * @return
	 */
	public static Point leftAdjacentPoint(Point A) {
		return new Point(A.x - 1, A.y, false);
	}

	/**
	 * Returns Right Adjacent Point
	 * @param A
	 * @return
	 */
	public static Point rightAdjacentPoint(Point A) {
		return new Point(A.x + 1, A.y, false);
	}

	/**
	 * Returns Upper Adjacent Point
	 * @param A
	 * @return
	 */
	public static Point upAdjacentPoint(Point A) {
		return new Point(A.x, A.y + 1, false);
	}

	/**
	 * Returns Lower Adjacent Point
	 * @param A
	 * @return
	 */
	public static Point downAdjacentPoint(Point A) {
		return new Point(A.x, A.y - 1, false);
	}
}