public class Obstacles {
	int x;
	int y;

	public Obstacles(int x, int y) {
		this.x = x;
		this.y = y;
	}

	@Override
	public String toString() {
		return "\n(X: " + Integer.toString(x) + ", Y: " + Integer.toString(y) + ")";
	}

}
