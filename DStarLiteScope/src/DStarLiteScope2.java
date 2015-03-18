import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.awt.GridLayout;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.List;
import java.util.Scanner;

import javax.swing.JButton;
import javax.swing.JFrame;
import java.util.*;

/**
 * Class DStarLiteScope2 with a limited scope. You are going towards an object
 * but you are only allowed to see obstacles within 1 square from you.
 * 
 * @author Aakash Parikh Justin Lin Kevin Wang
 *
 *         Based on code of Daniel Beard and D*Lite
 */
public class DStarLiteScope2 implements java.io.Serializable {
	public static ArrayList<Obstacles> obstaclesList = new ArrayList<Obstacles>();
	public List<State> traveledPath = new ArrayList<State>();
	public List<State> path = new ArrayList<State>();
	public PriorityQueue<State> openList = new PriorityQueue<State>();
	public HashMap<State, CellInfo> cellHash = new HashMap<State, CellInfo>();
	public HashMap<State, Float> openHash = new HashMap<State, Float>();

	// Private Member variables

	static int sX;
	static int sY;
	static int eX;
	static int eY;
	static int cX;
	static int cY;

	public double C1;
	public double k_m;
	public State s_start = new State();
	public State s_goal = new State();
	public State s_last = new State();
	public State s_current = new State();
	public int maxSteps;
	static long begin;

	public static double M_SQRT2 = Math.sqrt(2.0);
	public static final double maxVisionRange = 1 * M_SQRT2;
	static final long MAX_TIME = 5000;

	// Default constructor
	public DStarLiteScope2() {
		maxSteps = 80000;
		C1 = 1;
	}

	/*
	 * Initialize Method
	 * 
	 * @params start and goal coordinates
	 */
	public void init(int sX, int sY, int gX, int gY, int cX, int cY) {
		cellHash.clear();
		path.clear();
		openHash.clear();
		while (!openList.isEmpty())
			openList.poll();

		k_m = 0;

		s_start.x = sX;
		s_start.y = sY;
		s_goal.x = gX;
		s_goal.y = gY;

		CellInfo tmp = new CellInfo();
		tmp.g = 0;
		tmp.rhs = 0;
		tmp.cost = C1;

		cellHash.put(s_goal, tmp);

		tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(s_start, s_goal);
		tmp.cost = C1;
		cellHash.put(s_start, tmp);
		s_start = calculateKey(s_start);

		s_last = s_start;
		createWalls(cX, cY);

	}

	/*
	 * CalculateKey(state u) As per [S. Koenig, 2002]
	 */
	public State calculateKey(State u) {
		double val = Math.min(getRHS(u), getG(u));

		u.k.setFirst(val + heuristic(u, s_start) + k_m);
		u.k.setSecond(val);

		return u;
	}

	/*
	 * Returns the rhs value for state u.
	 */
	public double getRHS(State u) {
		if (u == s_goal)
			return 0;

		// if the cellHash doesn't contain the State u
		if (cellHash.get(u) == null)
			return heuristic(u, s_goal);
		return cellHash.get(u).rhs;
	}

	/*
	 * Returns the g value for the state u.
	 */
	public double getG(State u) {
		// if the cellHash doesn't contain the State u
		if (cellHash.get(u) == null)
			return heuristic(u, s_goal);
		return cellHash.get(u).g;
	}

	/*
	 * Pretty self explanatory, the heuristic we use is the 8-way distance
	 * scaled by a constant C1 (should be set to <= min cost)
	 */
	public double heuristic(State a, State b) {
		return eightCondist(a, b) * C1;
	}

	/*
	 * Returns the 8-way distance between state a and state b
	 */
	public double eightCondist(State a, State b) {
		double temp;
		double min = Math.abs(a.x - b.x);
		double max = Math.abs(a.y - b.y);
		if (min > max) {
			temp = min;
			min = max;
			max = temp;
		}
		return ((M_SQRT2 - 1.0) * min + max);

	}

	public boolean calculatePath() {
		// System.out.println("START PATH CALC");
		path.clear();

		int res = computeShortestPath();
		if (res < 0) {
			System.out.println("No Path to Goal");
			return false;
		}

		LinkedList<State> n = new LinkedList<State>();
		State cur = s_start;

		if (getG(s_start) == Double.POSITIVE_INFINITY) {
			System.out.println("No Path to Goal");
			return false;
		}
		boolean running = true;
		while (cur.neq(s_goal) && trueDist(s_start, cur) < maxVisionRange) {
			path.add(cur);

			traveledPath.add(new State(cur));
			n = new LinkedList<State>();
			n = getSucc(cur);

			if (n.isEmpty()) {
				System.out.println("No Path to Goal");
				return false;
			}

			double cmin = Double.POSITIVE_INFINITY;
			double tmin = 0;
			State smin = new State();

			for (State i : n) {
				double val = getRHS(i);
				double val2 = trueDist(i, s_goal) + trueDist(s_start, i);

				if (close(val, cmin)) {
					if (tmin > val2) {
						tmin = val2;
						cmin = val;
						smin = i;
					}
				} else if (val < cmin) {
					tmin = val2;
					cmin = val;
					smin = i;
				}
			}
			n.clear();
			cur = new State(smin);
		}

		if (cur.eq(s_goal)) {
			s_current = cur;
			traveledPath.add(new State(s_goal));
		} else {
			s_current = cur;
			updateStart(cur.x, cur.y);
		}

		return true;
	}

	/*
	 * As per [S. Koenig,2002] except for two main modifications: 1. We stop
	 * planning after a number of steps, 'maxsteps' we do this because this
	 * algorithm can plan forever if the start is surrounded by obstacles 2. We
	 * lazily remove states from the open list so we never have to iterate
	 * through it.
	 */
	public int computeShortestPath() {
		LinkedList<State> s = new LinkedList<State>();

		if (openList.isEmpty())
			return 1;

		int k = 0;

		while ((!openList.isEmpty())
				&& (openList.peek().lt(s_start = calculateKey(s_start)) || (getRHS(s_start) != getG(s_start)))) {

			if (k++ > maxSteps) {
				System.out.println("At maxsteps");
				return -1;
			}

			State u;

			boolean test = (getRHS(s_start) != getG(s_start));

			// lazy remove
			while (true) {
				if (openList.isEmpty())
					return 1;
				u = openList.poll();
				// System.out.println("OPEN LIST SIZE:"+openList.size());

				if (!isValid(u))
					continue;
				if (!(u.lt(s_start)) && (!test))
					return 2;
				break;
			}

			openHash.remove(u);

			State k_old = new State(u);

			/*
			 * if (k_old.lt(calculateKey(u))) { // u is out of date insert(u);
			 * System.out.println("CHOICE 1"); } else
			 */
			if (getG(u) > getRHS(u)) { // needs update (got better)
				setG(u, getRHS(u));
				s = getPred(u);
				for (State i : s) {
					updateVertex(i);
				}
			} else if (getG(u) < getRHS(u)) {// , state has got worse
				setG(u, Double.POSITIVE_INFINITY);
				s = getPred(u);

				for (State i : s) {
					updateVertex(i);
				}
				updateVertex(u);
			}
		}
		return 0;
	}

	/*
	 * Returns a list of successor states for state u, since this is an 8-way
	 * graph this list contains all of a cells neighbours. Unless the cell is
	 * occupied, in which case it has no successors.
	 */
	public LinkedList<State> getSucc(State u) {
		LinkedList<State> s = new LinkedList<State>();
		State tempState;
		if (occupied(u))
			return s;
		// Generate the successors, starting at the immediate right,
		// Moving in a clockwise manner
		tempState = new State(u.x + 1, u.y, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x + 1, u.y + 1, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x, u.y + 1, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x - 1, u.y + 1, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x - 1, u.y, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x - 1, u.y - 1, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x, u.y - 1, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x + 1, u.y - 1, new Pair(-1.0, -1.0));
		s.addFirst(tempState);

		return s;
	}

	/*
	 * Returns a list of all the predecessor states for state u. Since this is
	 * for an 8-way connected graph, the list contains all the neighbours for
	 * state u. Occupied neighbors are not added to the list
	 */
	public LinkedList<State> getPred(State u) {
		LinkedList<State> s = new LinkedList<State>();
		State tempState;

		tempState = new State(u.x + 1, u.y, new Pair(-1.0, -1.0));
		if (!occupied(tempState)) {
			s.addFirst(tempState);
		}

		tempState = new State(u.x + 1, u.y + 1, new Pair(-1.0, -1.0));
		if (!occupied(tempState)) {

			s.addFirst(tempState);
		}

		tempState = new State(u.x, u.y + 1, new Pair(-1.0, -1.0));
		if (!occupied(tempState)) {
			s.addFirst(tempState);
		}

		tempState = new State(u.x - 1, u.y + 1, new Pair(-1.0, -1.0));
		if (!occupied(tempState)) {
			s.addFirst(tempState);
		}

		tempState = new State(u.x - 1, u.y, new Pair(-1.0, -1.0));
		if (!occupied(tempState)) {
			s.addFirst(tempState);
		}

		tempState = new State(u.x - 1, u.y - 1, new Pair(-1.0, -1.0));

		if (!occupied(tempState)) {
			s.addFirst(tempState);
		}
		tempState = new State(u.x, u.y - 1, new Pair(-1.0, -1.0));
		if (!occupied(tempState)) {
			s.addFirst(tempState);
		}
		tempState = new State(u.x + 1, u.y - 1, new Pair(-1.0, -1.0));
		if (!occupied(tempState)) {
			s.addFirst(tempState);
		}
		return s;
	}

	/*
	 * Update the position of the agent/robot. This does not force a
	 * calculatePath.
	 */
	public void updateStart(int x, int y) {
		s_start.x = x;
		s_start.y = y;

		s_start = calculateKey(s_start);
		s_last = s_start;

	}

	/*
	 * This is somewhat of a hack, to change the position of the goal we first
	 * save all of the non-empty nodes on the map, clear the map, move the goal
	 * and add re-add all of the non-empty cells. Since most of these cells are
	 * not between the start and goal this does not seem to hurt performance too
	 * much. Also, it frees up a good deal of memory we are probably not going
	 * to use.
	 */
	public void updateGoal(int x, int y) {
		List<Pair<point, Double>> toAdd = new ArrayList<Pair<point, Double>>();
		Pair<point, Double> tempPoint;

		for (Map.Entry<State, CellInfo> entry : cellHash.entrySet()) {
			if (!close(entry.getValue().cost, C1)) {
				tempPoint = new Pair(new point(entry.getKey().x,
						entry.getKey().y), entry.getValue().cost);
				toAdd.add(tempPoint);
			}
		}

		cellHash.clear();
		openHash.clear();

		while (!openList.isEmpty())
			openList.poll();

		k_m = 0;

		s_goal.x = x;
		s_goal.y = y;

		CellInfo tmp = new CellInfo();
		tmp.g = tmp.rhs = 0;
		tmp.cost = C1;

		cellHash.put(s_goal, tmp);

		tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(s_start, s_goal);
		tmp.cost = C1;
		cellHash.put(s_start, tmp);
		s_start = calculateKey(s_start);

		s_last = s_start;

		Iterator<Pair<point, Double>> iterator = toAdd.iterator();
		while (iterator.hasNext()) {
			tempPoint = iterator.next();
			updateCell(tempPoint.first().x, tempPoint.first().y,
					tempPoint.second());
		}

	}

	public void updateVertex(State u) {
		LinkedList<State> s = new LinkedList<State>();

		if (u.neq(s_goal)) {
			s = getSucc(u);
			double tmp = Double.POSITIVE_INFINITY;
			double tmp2;

			for (State i : s) {
				tmp2 = getG(i) + cost(u, i);
				if (tmp2 < tmp)
					tmp = tmp2;
			}
			if (!close(getRHS(u), tmp))
				setRHS(u, tmp);
		}

		if (!close(getG(u), getRHS(u)))
			insert(u);
	}

	/*
	 * Returns true if state u is on the open list or not by checking if it is
	 * in the hash table.
	 */
	public boolean isValid(State u) {
		if (openHash.get(u) == null)
			return false;
		if (!close(keyHashCode(u), openHash.get(u)))
			return false;
		return true;
	}

	/*
	 * Sets the G value for state u
	 */
	public void setG(State u, double g) {
		makeNewCell(u);
		cellHash.get(u).g = g;
	}

	/*
	 * Sets the rhs value for state u
	 */
	public void setRHS(State u, double rhs) {
		makeNewCell(u);
		cellHash.get(u).rhs = rhs;
	}

	/*
	 * Checks if a cell is in the hash table, if not it adds it in.
	 */
	public void makeNewCell(State u) {
		if (cellHash.get(u) != null)
			return;
		CellInfo tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(u, s_goal);
		tmp.cost = C1;
		cellHash.put(u, tmp);
	}

	public void markImpassable(int x, int y) {
		State u = new State();
		u.x = x;
		u.y = y;

		CellInfo tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(u, s_goal);
		tmp.cost = -1;
		cellHash.put(u, tmp);
		setG(u, Double.POSITIVE_INFINITY);

	}

	/*
	 * updateCell as per [S. Koenig, 2002]
	 */
	public void updateCell(int x, int y, double val) {
		State u = new State();
		u.x = x;
		u.y = y;

		if ((u.eq(s_start)) || (u.eq(s_goal)))
			return;

		makeNewCell(u);
		cellHash.get(u).cost = val;
		/*
		 * if(val < 0){ cellHash.get(u).markAsInvalid(); }
		 */
		updateVertex(u);
	}

	/*
	 * Inserts state u into openList and openHash
	 */
	public void insert(State u) {
		// iterator cur
		float csum;

		u = calculateKey(u);
		// cur = openHash.find(u);
		csum = keyHashCode(u);

		// return if cell is already in list. TODO: this should be
		// uncommented except it introduces a bug, I suspect that there is a
		// bug somewhere else and having duplicates in the openList queue
		// hides the problem...
		// if ((cur != openHash.end()) && (close(csum,cur->second))) return;

		openHash.put(u, csum);
		openList.add(u);
	}

	/*
	 * Returns the key hash code for the state u, this is used to compare a
	 * state that has been updated
	 */
	public float keyHashCode(State u) {
		return (float) (u.k.first() + 1193 * u.k.second());
	}

	/*
	 * Returns true if the cell is occupied (non-traversable), false otherwise.
	 * Non-traversable are marked with a cost < 0
	 */
	public boolean occupied(State u) {
		// if the cellHash does not contain the State u
		if (cellHash.get(u) == null)
			return false;
		if (!cellHash.get(u).valid) {
			return false;
		}
		return (cellHash.get(u).cost < 0);

		/*
		 * if(u.x > eX){ System.out.println("2"); return true; }else if (u.y >
		 * eY){ System.out.println("3"); return true; }else if (u.x<sX){
		 * System.out.println("4"); return true; }else if (u.y < sY){
		 * System.out.println("5"); return true; }else if(isValid(u)){ if
		 * (cellHash.get(u).cost < 0){ System.out.println("6"); return false;
		 * }else{ return false; } }else{ System.out.println("7"); return false;
		 * }
		 */
	}

	/*
	 * Euclidean cost between state a and state b
	 */
	public double trueDist(State a, State b) {
		float x = a.x - b.x;
		float y = a.y - b.y;
		return Math.sqrt(x * x + y * y);
	}

	/*
	 * Returns the cost of moving from state a to state b. This could be either
	 * the cost of moving off state a or onto state b, we went with the former.
	 * This is also the 8-way cost.
	 */
	public double cost(State a, State b) {
		if (cellHash.get(a) != null && cellHash.get(b) != null) {
			if (cellHash.get(a).cost < 0 || cellHash.get(b).cost < 0) {
				return Double.POSITIVE_INFINITY;
			}
		}
		int xd = Math.abs(a.x - b.x);
		int yd = Math.abs(a.y - b.y);
		double scale = 1;

		if (xd + yd > 1)
			scale = M_SQRT2;

		if (cellHash.containsKey(a) == false)
			return scale * C1;
		return scale * cellHash.get(a).cost;
	}

	/*
	 * Returns true if x and y are within 10E-5, false otherwise
	 */
	public boolean close(double x, double y) {
		if (x == Double.POSITIVE_INFINITY && y == Double.POSITIVE_INFINITY)
			return true;
		return (Math.abs(x - y) < 0.00001);
	}

	public List<State> getPath() {
		return path;
	}

	public void createWalls(int x, int y) { // bottomleftmost point
		for (int i = -1; i < x + 2; i++) {
			updateCell(i, y + 1, -1);
			updateCell(i, -1, -1);
		}
		for (int i = -1; i < y + 2; i++) {
			updateCell(x + 1, i, -1);
			updateCell(-1, i, -1);
		}
	}

	public static void main(String[] args) {
		DStarLiteScope2 pf = new DStarLiteScope2();

		// Adding obstacles an creating the maze
		Scanner sc = null;
		try {
			sc = new Scanner(new File("obstacles.txt"));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		begin = System.currentTimeMillis();
		sX = sc.nextInt();
		sY = sc.nextInt();
		eX = sc.nextInt();
		eY = sc.nextInt();
		cX = sc.nextInt();
		cY = sc.nextInt();
		pf.init(sX, sY, eX, eY, cX, cY);
		// pf.createWalls(10, 10); // (xmax, //ymax)
		int numOfObstacles = sc.nextInt();
		for (int i = 0; i < numOfObstacles; i++) {
			while (sc.hasNext()) {
				int x = sc.nextInt();
				int y = sc.nextInt();
				Obstacles temp = new Obstacles(x, y);
				obstaclesList.add(temp);
			}
		}
		boolean running = true;
		while (pf.s_current.neq(pf.s_goal) && (running)) {
			pf.updateObstacles(obstaclesList);
			running = pf.calculatePath();
		}

		List<State> path = pf.getPath();

		// Calculate distance
		double distance = 0;
		int prevX = sX;
		int prevY = sY;
		System.out.println(pf.traveledPath.toString());
		for (State i : pf.traveledPath) {
			distance += Math.sqrt(Math.pow(i.x - prevX, 2)
					+ Math.pow(i.y - prevY, 2));
			prevX = i.x;
			prevY = i.y;
		}
		System.out.println();
		System.out.println();
		System.out.println("Start node: (" + sX + "," + sY + ")");
		System.out.println("End node: (" + eX + "," + eY + ")");
		System.out.println("Coordinate Size: " + cX * cY + " X: 0 to " + cX
				+ " Y: 0 to " + cY);
		System.out.println("You can see " + pf.maxVisionRange + "units");
		System.out.println();
		System.out.println();
		System.out.println();
		System.out.println();
		System.out.println("Steps = " + (pf.traveledPath.size() - 1));
		long end = System.currentTimeMillis();
		System.out.println("Time: " + (end - begin) + "ms");
		System.out.println("Distance: " + distance);
		System.out.println("Number of Obstacles: " + numOfObstacles + ": ");
		System.out.println(obstaclesList.toString());
		pf.draw();

	}

	public void updateObstacles(ArrayList<Obstacles> obstaclesList) {
		for (Obstacles o : obstaclesList) {
			// System.out.println("SX:" + s_start.x+" SY:" + s_start.y);
			if (Math.sqrt(Math.pow(o.x - s_start.x, 2)
					+ Math.pow(o.y - s_start.y, 2)) <= maxVisionRange) {
				// System.out.println("OBS: " + o.x+","+o.y);
				// setG(, Double.POSITIVE_INFINITY);
				// markImpassable(o.x, o.y);
				updateCell(o.x, o.y, -1);

			}
		}
	}

	public void draw() {

		JFrame.setDefaultLookAndFeelDecorated(true);
		JFrame frame = new JFrame("Draw");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setLayout(new GridLayout(cX, cY));
		for (int n = 0; n < obstaclesList.size(); n++) {
		for (int i = 0; i < cX * cY; i++) {
			if (i % cX == sX && i / cY == sY) {
				frame.add(new JButton("S"));
			} else if (i % cX == eX && i / cY == eY) {
				frame.add(new JButton("E"));
			} else if(i % cX == obstaclesList.get(n).x
							&& i / obstaclesList.get(n).y == sY) {
						frame.add(new JButton("O"));
					}
				}
			}

		System.out.println(cX);
		System.out.println(cY);
		frame.pack();
		frame.setVisible(true);
	}
}

class CellInfo implements java.io.Serializable {
	public double g = 0;
	public double rhs = 0;
	public double cost = 0;
	public boolean valid = true;

	public void markAsInvalid() {
		valid = false;
		g = Double.POSITIVE_INFINITY;
	}
}

class point {
	public int x = 0;
	public int y = 0;

	public point() {
	}

	public point(int x, int y) {
		this.x = x;
		this.y = y;
	}
}