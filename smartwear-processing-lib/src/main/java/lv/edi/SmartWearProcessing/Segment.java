package lv.edi.SmartWearProcessing;
import java.util.Vector;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

/**
 * class that describes one sensor grid segment (node or accelerometer)
 * @author Richards
 *
 */
public class Segment {
	/**
	 * initial cross point coordinates (before vertical axis compensation)
	 * 
	 * 				P0
	 *              |
	 *              |
	 *    p3________|center_____P1
	 *             	|
	 *              |
	 *              |
	 *              p2
	 *              
	 *              iicross[0]=p0
	 *              iicross[1]=p1
	 *              iicross[2]=p2
	 *              iicross[3]=p3
	 */
	private float verticalDistance, horizontalDistance;
	public float[][] iicross={{0,0, (float)2.25},{(float)1.75,0,0},{0,0,(float)-2.25},{(float)-1.75,0,0}}; // initial cross consisting of 4 vectors depends on distance from sensors
	/**
	 * initial cross after rotation around vertical axis compensation
	 */
	public float[][] initialCross={{0,0, (float)2.25},{(float)1.75,0,0},{0,0,(float)-2.25},{(float)-1.75,0,0}}; // initial cross consisting of 4 vectors depends on distance from sensors
	/**
	 * cross that is oriented as sensor (represents real time orientation)
	 */
	public float[][] cross = {{0,0, (float)2.25},{(float)1.75,0,0},{0,0,(float)-2.25},{(float)-1.75,0,0}};;			   // rotated cross
	/**
	 * center coordinate of sensor
	 */
	public float[] center={0,0,0};				   // center coordinates of segment
	/**
	 * constructor
	 */
	
	public Segment(){
		
	}
	public Segment(float angle){
		float[] q = new float[4];
		float[] n = {0, 0, 1};
		float tempCross[][] = new float[4][3];
		for(int i=0;i<4;i++){
			for(int j=0;j<3;j++){
				tempCross[i][j]=initialCross[i][j];
			}
		}
		SensorDataProcessing.quaternion(n, angle, q);
		for(int i = 0; i<4; i++){
			SensorDataProcessing.quatRotate(q, tempCross[i], initialCross[i]);
		}	
	}
	/**Sets segment current orientation 
	 * @param sensor sensor of corresponding sensor
	 * */
	public void setSegmentOrientation(Sensor sensor){
		float[] n = new float[3]; // rotation axis
		float[] vec = {0, 0, 1};  // Z axis
		float fi = 0; 			  // rotation angle
		float[] q = new float[4];// quaternion
		float[] accnorm = sensor.getAccNorm();
		SensorDataProcessing.crossProduct(sensor.getAccNorm(),vec, n); // rotation axis
		SensorDataProcessing.normalizeVector(n);					      // normalize rotation axis				
		fi = (float)Math.acos(SensorDataProcessing.dotProduct(vec, sensor.getAccNorm())); // get rotation angle
		SensorDataProcessing.quaternion(n, fi, q); //get quaternion
		for(int i = 0; i<4; i++){
			SensorDataProcessing.quatRotate(q, initialCross[i], cross[i]);
			
		}
	}
	/** sets segment current orientation using TRIAD algorithm
	 * 
	 * @param sensor sensor of corresponding sensor
	 */
	public void setSegmentOrientationTRIAD(Sensor sensor){
		float[][] rotMat=SensorDataProcessing.getRotationTRIAD(sensor.getAccNorm(), sensor.getMagNorm());
		// rotate each segment
		for(int i = 0; i<4; i++){
			SensorDataProcessing.multiplyMatrix(rotMat, initialCross[i], cross[i]);
		}
	}


	/**
	 * Sets all segment orientation with triad algorithm defined in segment array
	 * @param segmentArray array of segments
	 * @param sensorGrid grid of sensors
	 */
	public static void setAllSegmentOrientationsTRIAD(Segment[][] segmentArray, Sensor[][] sensorGrid){
		for(int i=0; i<segmentArray.length; i++){
			for(int j=0; j<segmentArray[0].length; j++){
				segmentArray[i][j].setSegmentOrientationTRIAD(sensorGrid[i][j]);
			}
		}
	}

	public static void setAllSegmentOrientationsTRIAD(Vector<Vector <Segment>> segmentGrid, Vector<Vector <Sensor>> sensorGrid){
		for(int i=0; i<segmentGrid.size(); i++){
			for(int j=0; j<segmentGrid.get(0).size(); j++){
				segmentGrid.get(i).get(j).setSegmentOrientationTRIAD(sensorGrid.get(i).get(j));
			}
		}
	}
	
	/**
	 * Sets all segment orientations defined in segment array
	 * @param segmentArray
	 * @param sensorGrid
	 */
	public static void setAllSegmentOrientations(Segment[][] segmentArray,Sensor[][] sensorGrid){
		for(int i=0; i<segmentArray.length; i++){
			for(int j=0; j<segmentArray[0].length; j++){
				segmentArray[i][j].setSegmentOrientation(sensorGrid[i][j]);
			}
		}
	}


	/**
	 * Function sets segment orientation where segments and sensors are stored in
	 * Vector<Vector<T>> type 2D structure
	 * @param segments array containing segments objects
	 * @param sensors array containing sensor objects
	 */
	public static void setAllSegmentOrientations(Vector<Vector<Segment>> segments, Vector<Vector<Sensor>>sensors){
		for(int i=0; i<segments.size(); i++){
			for(int j=0; j<segments.get(i).size(); j++){
				segments.get(i).get(j).setSegmentOrientation(sensors.get(i).get(j));
			}
		}
	}
	/**
	 * method sets segment rotation around vertical axis
	 * @param angle - rotation angle in radians
	 */
	public void setSegmentZRotation(float angle){
		float[] q = new float[4];
		float[] n = {0, 0, 1};
		SensorDataProcessing.quaternion(n, angle, q);
		float[][] tempCross = new float[4][3];
		for(int i=0;i<4;i++){
			for(int j=0;j<3;j++){
				tempCross[i][j]=iicross[i][j];
			}
		}
		for(int i=0; i<4; i++){
			SensorDataProcessing.quatRotate(q, tempCross[i], initialCross[i]);
		}
	}
	/** sets initial cross vectors @args verticDistance - vertical distance between sensors horizDistance - horizontal ditances between sensors*/
	public void setInitialCross(float verticDistance, float horizDistance){
		initialCross[0][0]=0;
		initialCross[0][1]=0;
		initialCross[0][2]=verticDistance/2;

		initialCross[1][0]=0;
		initialCross[1][1]=horizDistance/2;
		initialCross[1][2]=0;

		initialCross[2][0]=0;
		initialCross[2][1]=0;
		initialCross[2][2]=-verticDistance/2;

		initialCross[3][0]=0;
		initialCross[3][1]=-horizDistance/2;
		initialCross[3][2]=0;
	}

	/** sets initial cross vectors @args verticDistance - vertical distance between sensors horizDistance - horizontal ditances between sensors*/
	public void setInitialCross2(float verticDistance, float horizDistance){
		this.verticalDistance=verticDistance;
		this.horizontalDistance=horizDistance;
		initialCross[0][0]=0;
		initialCross[0][1]=0;
		initialCross[0][2]=verticDistance/2;

		initialCross[1][0]=horizDistance/2;
		initialCross[1][1]=0;
		initialCross[1][2]=0;

		initialCross[2][0]=0;
		initialCross[2][1]=0;
		initialCross[2][2]=-verticDistance/2;

		initialCross[3][0]=-horizDistance/2;
		initialCross[3][1]=0;
		initialCross[3][2]=0;

		initialCross[0][0]=0;
		initialCross[0][1]=0;
		initialCross[0][2]=verticDistance/2;
	}
	/**method sets segment center coordinates for segment grid @args segmentArray is array of segments , referenceRow
	 * is row index for accelerometer from which to draw, referenceColumns is column index for accelerometer from which 
	 * to draw*/
	public static void setSegmentCenters(Segment[][] segmentArray, short referenceRow, short referenceCol){
		// calculating row
		short NR_ROWS = (short)segmentArray.length;
		short NR_COLS = (short)segmentArray[0].length;

		float[][][] tempCentersR = new float[NR_ROWS][NR_COLS][3];

		short toBottom=referenceRow;
		// solve centers for reference column
		while(toBottom>0){
			toBottom--;
			tempCentersR[toBottom][referenceCol][0]=tempCentersR[toBottom+1][referenceCol][0]+
														 segmentArray[toBottom+1][referenceCol].cross[2][0]+
														 segmentArray[toBottom][referenceCol].cross[2][0]; // X coordinate for segment center
			tempCentersR[toBottom][referenceCol][1]=tempCentersR[toBottom+1][referenceCol][1]+
					 									 segmentArray[toBottom+1][referenceCol].cross[2][1]+
					 									 segmentArray[toBottom][referenceCol].cross[2][1]; // Y coordinate for segment center
			tempCentersR[toBottom][referenceCol][2]=tempCentersR[toBottom+1][referenceCol][2]+
														 segmentArray[toBottom+1][referenceCol].cross[2][2]+
														 segmentArray[toBottom][referenceCol].cross[2][2]; // Z coordinate for segment center
		}
		short toTop=referenceRow;
		while(toTop<(NR_ROWS-1)){
			toTop++;
			tempCentersR[toTop][referenceCol][0]=tempCentersR[toTop-1][referenceCol][0]+
														  segmentArray[toTop-1][referenceCol].cross[0][0]+
														  segmentArray[toTop][referenceCol].cross[0][0];// X coordinate for segment center		
			tempCentersR[toTop][referenceCol][1]=tempCentersR[toTop-1][referenceCol][1]+
					 									  segmentArray[toTop-1][referenceCol].cross[0][1]+
					 									  segmentArray[toTop][referenceCol].cross[0][1];// Y coordinate for segment center
			tempCentersR[toTop][referenceCol][2]=tempCentersR[toTop-1][referenceCol][2]+
														  segmentArray[toTop-1][referenceCol].cross[0][2]+
														  segmentArray[toTop][referenceCol].cross[0][2];// Z coordinate for segment center											 
		}
		// calculating centres for rest of the segments row-wise
		
		for(int i=0;i<NR_ROWS;i++){
			short toLeft=referenceCol;
			while(toLeft<(NR_COLS-1)){
				toLeft++;
				tempCentersR[i][toLeft][0]=tempCentersR[i][toLeft-1][0]+
													segmentArray[i][toLeft-1].cross[3][0]+
													segmentArray[i][toLeft].cross[3][0]; // X coordinate
				tempCentersR[i][toLeft][1]=tempCentersR[i][toLeft-1][1]+
													segmentArray[i][toLeft-1].cross[3][1]+
													segmentArray[i][toLeft].cross[3][1]; // Y coordinate
				tempCentersR[i][toLeft][2]=tempCentersR[i][toLeft-1][2]+
													segmentArray[i][toLeft-1].cross[3][2]+
													segmentArray[i][toLeft].cross[3][2]; // Z coordinate
			}
			short toRight=referenceCol;
			while(toRight>0){
				toRight--;
				tempCentersR[i][toRight][0]=tempCentersR[i][toRight+1][0]+
												 segmentArray[i][toRight+1].cross[1][0]+
												 segmentArray[i][toRight].cross[1][0]; // X coordinate
				tempCentersR[i][toRight][1]=tempCentersR[i][toRight+1][1]+
						 						 segmentArray[i][toRight+1].cross[1][1]+
						 						 segmentArray[i][toRight].cross[1][1]; // Y coordinate
				tempCentersR[i][toRight][2]=tempCentersR[i][toRight+1][2]+
						 						 segmentArray[i][toRight+1].cross[1][2]+
						 						 segmentArray[i][toRight].cross[1][2]; // X coordinate
			}
		}
		// calculating centres for reference row
		float[][][] tempCentersC = new float[NR_ROWS][NR_COLS][3];
		short toLeft=referenceCol;
		while(toLeft<(NR_COLS-1)){
			toLeft++;
			tempCentersC[referenceRow][toLeft][0]=tempCentersC[referenceRow][toLeft-1][0]+
														 segmentArray[referenceRow][toLeft-1].cross[3][0]+
														 segmentArray[referenceRow][toLeft].cross[3][0]; // X coordinate for segment center
			tempCentersC[referenceRow][toLeft][1]=tempCentersC[referenceRow][toLeft-1][1]+
					 									 segmentArray[referenceRow][toLeft-1].cross[3][1]+
					 									 segmentArray[referenceRow][toLeft].cross[3][1]; // Y coordinate for segment center
			tempCentersC[referenceRow][toLeft][2]=tempCentersC[referenceRow][toLeft-1][2]+
														 segmentArray[referenceRow][toLeft-1].cross[3][2]+
														 segmentArray[referenceRow][toLeft].cross[3][2]; // Z coordinate for segment center
		}
		short toRight=referenceCol;
		
		while(toRight>0){
			toRight--;
			tempCentersC[referenceRow][toRight][0]=tempCentersC[referenceRow][toRight+1][0]+
														  segmentArray[referenceRow][toRight+1].cross[1][0]+
														  segmentArray[referenceRow][toRight].cross[1][0];// X coordinate for segment center		
			tempCentersC[referenceRow][toRight][1]=tempCentersC[referenceRow][toRight+1][1]+
					 									  segmentArray[referenceRow][toRight+1].cross[1][1]+
					 									  segmentArray[referenceRow][toRight].cross[1][1];// Y coordinate for segment center
			tempCentersC[referenceRow][toRight][2]=tempCentersC[referenceRow][toRight+1][2]+
														  segmentArray[referenceRow][toRight+1].cross[1][2]+
														  segmentArray[referenceRow][toRight].cross[1][2];// Z coordinate for segment center											 
		}
		
		// calculating centres for rest of the segments columnwise

		for(int i=0;i<NR_COLS;i++){
			toBottom=referenceRow;
			while(toBottom>0){
				toBottom--;
				tempCentersC[toBottom][i][0]=tempCentersC[toBottom+1][i][0]+
													  segmentArray[toBottom+1][i].cross[2][0]+
													  segmentArray[toBottom][i].cross[2][0]; // X coordinate
				tempCentersC[toBottom][i][1]=tempCentersC[toBottom+1][i][1]+
													  segmentArray[toBottom+1][i].cross[2][1]+
													  segmentArray[toBottom][i].cross[2][1]; // Y coordinate
				tempCentersC[toBottom][i][2]=tempCentersC[toBottom+1][i][2]+
													  segmentArray[toBottom+1][i].cross[2][2]+
													  segmentArray[toBottom][i].cross[2][2]; // Z coordinate
			}
			toTop=referenceRow;
			while(toTop<(NR_ROWS-1)){
				toTop++;
				tempCentersC[toTop][i][0]=tempCentersC[toTop-1][i][0]+
													   segmentArray[toTop-1][i].cross[0][0]+
													   segmentArray[toTop][i].cross[0][0]; // X coordinate
				tempCentersC[toTop][i][1]=tempCentersC[toTop-1][i][1]+
								 					  segmentArray[toTop-1][i].cross[0][1]+
								 					  segmentArray[toTop][i].cross[0][1]; // Y coordinate
				tempCentersC[toTop][i][2]=tempCentersC[toTop-1][i][2]+
								 						 segmentArray[toTop-1][i].cross[0][2]+
								 						 segmentArray[toTop][i].cross[0][2]; // X coordinate
			}
		}
		// combine centres solved by columns and rows by computing average
		for(int i=0; i<segmentArray.length; i++){
			for(int j=0; j<segmentArray[0].length; j++){
				for(int coord=0; coord<3; coord++){
					segmentArray[i][j].center[coord]=(tempCentersR[i][j][coord]+tempCentersC[i][j][coord])/2;//+tempCentersR[i][j][coord])/2;
				}
			}
		}
	}

	/**method sets segment center coordinates for segment grid @args segmentArray is array of segments , referenceRow
	 * is row index for accelerometer from which to draw, referenceColumns is column index for accelerometer from which
	 * to draw*/
	public static void setSegmentCenters(Vector<Vector<Segment>> segmentArray, short referenceRow, short referenceCol){
		// calculating row
		short NR_ROWS = (short)segmentArray.size();
		short NR_COLS = (short)segmentArray.get(0).size();

		float[][][] tempCentersR = new float[NR_ROWS][NR_COLS][3];

		short toBottom=referenceRow;
		// solve centers for reference column
		while(toBottom>0){
			toBottom--;
			tempCentersR[toBottom][referenceCol][0]=tempCentersR[toBottom+1][referenceCol][0]+
					segmentArray.get(toBottom+1).get(referenceCol).cross[2][0]+
					segmentArray.get(toBottom).get(referenceCol).cross[2][0]; // X coordinate for segment center
			tempCentersR[toBottom][referenceCol][1]=tempCentersR[toBottom+1][referenceCol][1]+
					segmentArray.get(toBottom+1).get(referenceCol).cross[2][1]+
					segmentArray.get(toBottom).get(referenceCol).cross[2][1]; // Y coordinate for segment center
			tempCentersR[toBottom][referenceCol][2]=tempCentersR[toBottom+1][referenceCol][2]+
					segmentArray.get(toBottom+1).get(referenceCol).cross[2][2]+
					segmentArray.get(toBottom).get(referenceCol).cross[2][2]; // Z coordinate for segment center
		}
		short toTop=referenceRow;
		while(toTop<(NR_ROWS-1)){
			toTop++;
			tempCentersR[toTop][referenceCol][0]=tempCentersR[toTop-1][referenceCol][0]+
					segmentArray.get(toTop-1).get(referenceCol).cross[0][0]+
					segmentArray.get(toTop).get(referenceCol).cross[0][0];// X coordinate for segment center
			tempCentersR[toTop][referenceCol][1]=tempCentersR[toTop-1][referenceCol][1]+
					segmentArray.get(toTop-1).get(referenceCol).cross[0][1]+
					segmentArray.get(toTop).get(referenceCol).cross[0][1];// Y coordinate for segment center
			tempCentersR[toTop][referenceCol][2]=tempCentersR[toTop-1][referenceCol][2]+
					segmentArray.get(toTop-1).get(referenceCol).cross[0][2]+
					segmentArray.get(toTop).get(referenceCol).cross[0][2];// Z coordinate for segment center
		}
		// calculating centres for rest of the segments row-wise

		for(int i=0;i<NR_ROWS;i++){
			short toLeft=referenceCol;
			while(toLeft<(NR_COLS-1)){
				toLeft++;
				tempCentersR[i][toLeft][0]=tempCentersR[i][toLeft-1][0]+
						segmentArray.get(i).get(toLeft-1).cross[3][0]+
						segmentArray.get(i).get(toLeft).cross[3][0]; // X coordinate
				tempCentersR[i][toLeft][1]=tempCentersR[i][toLeft-1][1]+
						segmentArray.get(i).get(toLeft-1).cross[3][1]+
						segmentArray.get(i).get(toLeft).cross[3][1]; // Y coordinate
				tempCentersR[i][toLeft][2]=tempCentersR[i][toLeft-1][2]+
						segmentArray.get(i).get(toLeft-1).cross[3][2]+
						segmentArray.get(i).get(toLeft).cross[3][2]; // Z coordinate
			}

			short toRight=referenceCol;
			while(toRight>0){
				toRight--;
				tempCentersR[i][toRight][0]=tempCentersR[i][toRight+1][0]+
						segmentArray.get(i).get(toRight+1).cross[1][0]+
						segmentArray.get(i).get(toRight).cross[1][0]; // X coordinate
				tempCentersR[i][toRight][1]=tempCentersR[i][toRight+1][1]+
						segmentArray.get(i).get(toRight+1).cross[1][1]+
						segmentArray.get(i).get(toRight).cross[1][1]; // Y coordinate
				tempCentersR[i][toRight][2]=tempCentersR[i][toRight+1][2]+
						segmentArray.get(i).get(toRight+1).cross[1][2]+
						segmentArray.get(i).get(toRight).cross[1][2]; // X coordinate
			}
		}
		// calculating centres for reference row
		float[][][] tempCentersC = new float[NR_ROWS][NR_COLS][3];
		short toLeft=referenceCol;
		while(toLeft<(NR_COLS-1)){
			toLeft++;
			tempCentersC[referenceRow][toLeft][0]=tempCentersC[referenceRow][toLeft-1][0]+
					segmentArray.get(referenceRow).get(toLeft-1).cross[3][0]+
					segmentArray.get(referenceRow).get(toLeft).cross[3][0]; // X coordinate for segment center
			tempCentersC[referenceRow][toLeft][1]=tempCentersC[referenceRow][toLeft-1][1]+
					segmentArray.get(referenceRow).get(toLeft-1).cross[3][1]+
					segmentArray.get(referenceRow).get(toLeft).cross[3][1]; // Y coordinate for segment center
			tempCentersC[referenceRow][toLeft][2]=tempCentersC[referenceRow][toLeft-1][2]+
					segmentArray.get(referenceRow).get(toLeft-1).cross[3][2]+
					segmentArray.get(referenceRow).get(toLeft).cross[3][2]; // Z coordinate for segment center
		}
		short toRight=referenceCol;

		while(toRight>0){
			toRight--;
			tempCentersC[referenceRow][toRight][0]=tempCentersC[referenceRow][toRight+1][0]+
					segmentArray.get(referenceRow).get(toRight+1).cross[1][0]+
					segmentArray.get(referenceRow).get(toRight).cross[1][0];// X coordinate for segment center
			tempCentersC[referenceRow][toRight][1]=tempCentersC[referenceRow][toRight+1][1]+
					segmentArray.get(referenceRow).get(toRight+1).cross[1][1]+
					segmentArray.get(referenceRow).get(toRight).cross[1][1];// Y coordinate for segment center
			tempCentersC[referenceRow][toRight][2]=tempCentersC[referenceRow][toRight+1][2]+
					segmentArray.get(referenceRow).get(toRight+1).cross[1][2]+
					segmentArray.get(referenceRow).get(toRight).cross[1][2];// Z coordinate for segment center
		}

		// calculating centres for rest of the segments columnwise

		for(int i=0;i<NR_COLS;i++){
			toBottom=referenceRow;
			while(toBottom>0){
				toBottom--;
				tempCentersC[toBottom][i][0]=tempCentersC[toBottom+1][i][0]+
						segmentArray.get(toBottom+1).get(i).cross[2][0]+
						segmentArray.get(toBottom).get(i).cross[2][0]; // X coordinate
				tempCentersC[toBottom][i][1]=tempCentersC[toBottom+1][i][1]+
						segmentArray.get(toBottom+1).get(i).cross[2][1]+
						segmentArray.get(toBottom).get(i).cross[2][1]; // Y coordinate
				tempCentersC[toBottom][i][2]=tempCentersC[toBottom+1][i][2]+
						segmentArray.get(toBottom+1).get(i).cross[2][2]+
						segmentArray.get(toBottom).get(i).cross[2][2]; // Z coordinate
			}
			toTop=referenceRow;
			while(toTop<(NR_ROWS-1)){
				toTop++;
				tempCentersC[toTop][i][0]=tempCentersC[toTop-1][i][0]+
						segmentArray.get(toTop-1).get(i).cross[0][0]+
						segmentArray.get(toTop).get(i).cross[0][0]; // X coordinate
				tempCentersC[toTop][i][1]=tempCentersC[toTop-1][i][1]+
						segmentArray.get(toTop-1).get(i).cross[0][1]+
						segmentArray.get(toTop).get(i).cross[0][1]; // Y coordinate
				tempCentersC[toTop][i][2]=tempCentersC[toTop-1][i][2]+
						segmentArray.get(toTop-1).get(i).cross[0][2]+
						segmentArray.get(toTop).get(i).cross[0][2]; // X coordinate
			}
		}
		// combine centres solved by columns and rows by computing average
		for(int i=0; i<segmentArray.size(); i++){
			for(int j=0; j<segmentArray.get(i).size(); j++){
				for(int coord=0; coord<3; coord++){
					segmentArray.get(i).get(j).center[coord]=(tempCentersR[i][j][coord]+tempCentersC[i][j][coord])/2;//+tempCentersR[i][j][coord])/2;
				}
			}
		}
	}
	/**
	 * method that returns X coordinate of segment centre
	 * @return X coordinate of sement center
	 */
	public synchronized float getSegmentCenterX(){
		return center[0];
	}
	/**
	 *  return Y coordinate of segment centre
	 * @return Y coordinate of segment centre
	 */
	public synchronized float getSegmentCenterY(){
		return center[1];
	}
	/**
	 * returns Z coordinate of segment centre
	 * @return Y coordinate of segment centre
	 */
	public synchronized float getSegmentCenterZ(){
		return center[2];
	}
	/**
	 * method that gives particular vector of segment
	 * @param nr segment number
	 * @return cross vector float[3]
	 */
	public float [] getCross(int nr){
		float[] crossVec = new float[3];
		for(int i=0;i<3;i++){
			crossVec[i]=cross[nr][i]; // copy cross array to cross vec
		}
		return crossVec;
	}
	/**
	 * method that sets centres of segment array to default
	 * @param segmentArray input array of Segments
	 */
	public static void setDefaultCenters(Segment[][] segmentArray){
		for(int i=0;i<segmentArray.length;i++){
			for(int j=0;j<segmentArray[0].length;j++){
				segmentArray[i][j].center[2]=5*i;
				segmentArray[i][j].center[0]=5*j;
			}
		}
	}

	/**
	 * Generates array of segments in predefined grid
	 * @param rows nr of rows
	 * @param cols nr of cols
	 * @return segment grid
	 */
	public static Vector<Vector<Segment>> generateSegmentArray(int rows, int cols){
		Vector<Vector<Segment>> segments= new Vector(rows);
		segments.setSize(rows);
		for(int i=0; i<rows; i++) {
			Vector<Segment> rowSeg = new Vector<Segment>(cols);
			rowSeg.setSize(cols);
			for (int j = 0; j < cols; j++) {
				Segment seg = new Segment();
				seg.center[2] = 5 * i;
				seg.center[0] = 5 * j;
				rowSeg.set(j, seg);
			}
			segments.set(i, rowSeg);
		}
		return segments;
	}
	/**
	 * method that compares two segment arrays by distance between segment centres
	 * @param referenceState - first state Segment[][] array
	 * @param currentState - second state Segment[][] array
	 * @return array of distances[][]
	 */
	public static float[][] compareByCenterDistances(Segment[][] referenceState, Segment[][] currentState){
		float[][] distances=new float[referenceState.length][referenceState[0].length];
		for(int i=0;i<referenceState.length;i++){
			for(int j=0;j<referenceState[0].length;j++){
				distances[i][j]=(float)sqrt(pow(referenceState[i][j].center[0]-currentState[i][j].center[0],2)+
											pow(referenceState[i][j].center[1]-currentState[i][j].center[1],2)+
											pow(referenceState[i][j].center[2]-currentState[i][j].center[2],2));
			}
		}
		return distances;
	}

	/**
	 * Method computes distances between two segment arrays. segment arrays are of form Vector.
	 * @param referenceState saved state segments
	 * @param currentState current state segments
	 * @param distances distances must be preallocated before
	 */
	public static void compareByDistances(Vector<Vector<Segment>> referenceState, Vector<Vector<Segment>> currentState, Vector<Vector<Float>> distances){

		for(int i=0;i<referenceState.size();i++){
			for(int j=0;j<referenceState.get(0).size();j++){
				distances.get(i).set(j,new Float((float)sqrt(pow(referenceState.get(i).get(j).center[0]-currentState.get(i).get(j).center[0],2)+
						pow(referenceState.get(i).get(j).center[1]-currentState.get(i).get(j).center[1],2)+
						pow(referenceState.get(i).get(j).center[2]-currentState.get(i).get(j).center[2],2))));
			}
		}
	}

	/**method that compares two segment arrays by distances between segment centres in sagital plane
	 * 
	 *@param referenceState - first state Segment[][] array
	 * @param currentState - second state Segment[][] array
	 * @return array of distances[][]
	 */
	public static float[][] compareByDistancesSagital(Segment[][] referenceState, Segment[][] currentState){
		float[][] distancesSagital = new float[referenceState.length][referenceState[0].length];
		for(int i=0;i<referenceState.length;i++){
			for(int j=0;j<referenceState[0].length;j++){
				distancesSagital[i][j]=(float)sqrt(pow(referenceState[i][j].center[1]-currentState[i][j].center[1],2)+
											       pow(referenceState[i][j].center[2]-currentState[i][j].center[2],2));
			}
		}
		return distancesSagital;
	}
	/**method that compares two segment arrays by distances between segment centres in coronal plane
	 * 
	 *@param referenceState - first state Segment[][] array
	 * @param currentState - second state Segment[][] array
	 * @return array of distances[][]
	 */
	public static float[][] compareByDistancesCoronal(Segment[][] referenceState, Segment[][] currentState){
		float[][] distancesCoronal = new float[referenceState.length][referenceState[0].length];
		for(int i=0;i<referenceState.length;i++){
			for(int j=0;j<referenceState[0].length;j++){
				distancesCoronal[i][j]=(float)sqrt(pow(referenceState[i][j].center[0]-currentState[i][j].center[0],2)+
											       pow(referenceState[i][j].center[2]-currentState[i][j].center[2],2));
			}
		}
		return distancesCoronal;
	}
	
	public static void compansateCentersForTilt(Segment[][] refferenceStateInitial, Segment[][] refferenceState, float[][] Rreference, float[][] Rcurent){
		float[][] R = SensorDataProcessing.multMatMatT(Rcurent, Rreference);
		
		//SensorDataProcessing.transpose(R);
		for(int i=0; i<refferenceState.length; i++){
			for(int j=0; j<refferenceState[0].length;j++){
				
				SensorDataProcessing.multiplyMatrix(R, refferenceStateInitial[i][j].center, refferenceState[i][j].center);
				
			}
		}
	}

	public static void compansateCentersForTilt(Vector<Vector<Segment>> refferenceStateInitial, Vector<Vector<Segment>> refferenceState, float[][] Rreference, float[][] Rcurent){
		float[][] R = SensorDataProcessing.multMatMatT(Rcurent, Rreference);

		//SensorDataProcessing.transpose(R);
		for(int i=0; i<refferenceState.size(); i++){
			for(int j=0; j<refferenceState.get(0).size();j++){

				SensorDataProcessing.multiplyMatrix(R, refferenceStateInitial.get(i).get(j).center, refferenceState.get(i).get(j).center);
			}
		}
	}
	/**
	 * method for compensating for tilt 
	 * @param refferenceStateInitial
	 * @param currentState
	 * @param refferenceState
	 * @param referenceRow
	 * @param referenceCol
	 */
	public static void compansateCentersForTilt(Segment[][] refferenceStateInitial, Segment[][] currentState, Segment[][] refferenceState, int referenceRow, int referenceCol){
		float n[] = new float[3];
		float n2[] = new float[3];
		float fi;
		float fi2;
		float[] q = new float[4];
		float[] q2 = new float[4];

		SensorDataProcessing.crossProduct(refferenceStateInitial[referenceRow][referenceCol].cross[0], currentState[referenceRow][referenceCol].cross[0], n);
		
		SensorDataProcessing.normalizeVector(n);
	
		fi=(float)Math.acos(SensorDataProcessing.dotProduct(refferenceStateInitial[referenceRow][referenceCol].cross[0], currentState[referenceRow][referenceCol].cross[0])/
				(SensorDataProcessing.getVectorLength(refferenceStateInitial[referenceRow][referenceCol].cross[0])*SensorDataProcessing.getVectorLength(currentState[referenceRow][referenceCol].cross[0])));
		
		SensorDataProcessing.quaternion(n, fi, q);
		
		for(int i=0; i<refferenceState.length; i++){
			for(int j=0; j<refferenceState[0].length;j++){
				SensorDataProcessing.quatRotate(q, refferenceStateInitial[i][j].center, refferenceState[i][j].center);
				for(int k=0; k<4; k++){
					SensorDataProcessing.quatRotate(q, refferenceStateInitial[i][j].cross[k], refferenceState[i][j].cross[k]);
				}
				
			}
		}
		SensorDataProcessing.crossProduct(refferenceState[referenceRow][referenceCol].cross[1], currentState[referenceRow][referenceCol].cross[1], n2);
		SensorDataProcessing.normalizeVector(n2);
		
		fi2=(float)Math.acos(SensorDataProcessing.dotProduct(refferenceState[referenceRow][referenceCol].cross[1], currentState[referenceRow][referenceCol].cross[1])/
				(SensorDataProcessing.getVectorLength(refferenceState[referenceRow][referenceCol].cross[1])*SensorDataProcessing.getVectorLength(currentState[referenceRow][referenceCol].cross[1])));
		SensorDataProcessing.quaternion(n2, fi2, q2);
		
		for(int i=0; i<refferenceState.length; i++){
			for(int j=0; j<refferenceState[0].length;j++){
				Segment temp = new Segment();
				for(int k=0; k<3; k++){
					temp.center[k]=refferenceState[i][j].center[k];
				}
				SensorDataProcessing.quatRotate(q2, temp.center, refferenceState[i][j].center);
			}
		}
	}

	/**
	 * method for compensating for tilt
	 * @param refferenceStateInitial
	 * @param currentState
	 * @param refferenceState
	 * @param referenceRow
	 * @param referenceCol
	 */
	public static void compansateCentersForTilt(Vector<Vector<Segment>> refferenceStateInitial, Vector<Vector<Segment>> currentState, Vector<Vector<Segment>> refferenceState, int referenceRow, int referenceCol){
		float n[] = new float[3];
		float n2[] = new float[3];
		float fi;
		float fi2;
		float[] q = new float[4];
		float[] q2 = new float[4];

		SensorDataProcessing.crossProduct(refferenceStateInitial.get(referenceRow).get(referenceCol).cross[0], currentState.get(referenceRow).get(referenceCol).cross[0], n);

		SensorDataProcessing.normalizeVector(n);

		fi=(float)Math.acos(SensorDataProcessing.dotProduct(refferenceStateInitial.get(referenceRow).get(referenceCol).cross[0], currentState.get(referenceRow).get(referenceCol).cross[0])/
				(SensorDataProcessing.getVectorLength(refferenceStateInitial.get(referenceRow).get(referenceCol).cross[0])*SensorDataProcessing.getVectorLength(currentState.get(referenceRow).get(referenceCol).cross[0])));

		SensorDataProcessing.quaternion(n, fi, q);

		for(int i=0; i<refferenceState.size(); i++){
			for(int j=0; j<refferenceState.get(0).size();j++){
				SensorDataProcessing.quatRotate(q, refferenceStateInitial.get(i).get(j).center, refferenceState.get(i).get(j).center);
				for(int k=0; k<4; k++){
					SensorDataProcessing.quatRotate(q, refferenceStateInitial.get(i).get(j).cross[k], refferenceState.get(i).get(j).cross[k]);
				}

			}
		}
		SensorDataProcessing.crossProduct(refferenceState.get(referenceRow).get(referenceCol).cross[1], currentState.get(referenceRow).get(referenceCol).cross[1], n2);
		SensorDataProcessing.normalizeVector(n2);

		fi2=(float)Math.acos(SensorDataProcessing.dotProduct(refferenceState.get(referenceRow).get(referenceCol).cross[1], currentState.get(referenceRow).get(referenceCol).cross[1])/
				(SensorDataProcessing.getVectorLength(refferenceState.get(referenceRow).get(referenceCol).cross[1])*SensorDataProcessing.getVectorLength(currentState.get(referenceRow).get(referenceCol).cross[1])));
		SensorDataProcessing.quaternion(n2, fi2, q2);

		for(int i=0; i<refferenceState.size(); i++){
			for(int j=0; j<refferenceState.get(0).size();j++){
				Segment temp = new Segment();
				temp.setInitialCross2(refferenceStateInitial.get(0).get(0).getVerticalDistance(), refferenceStateInitial.get(0).get(0).getHorizontalDistance());
				for(int k=0; k<3; k++){
					temp.center[k]=refferenceState.get(i).get(j).center[k];
				}
				SensorDataProcessing.quatRotate(q2, temp.center, refferenceState.get(i).get(j).center);
			}
		}
	}

	public float getVerticalDistance(){
		return verticalDistance;
	}
	public float getHorizontalDistance(){
		return horizontalDistance;
	}
}
