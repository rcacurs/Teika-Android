package lv.edi.SmartWearProcessing;

import java.util.Vector;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

/**This class contains static functions for sensor data processing
 * @author Richards Cacurs*/
public class SensorDataProcessing {
	//method that calculates each accelerometer segment position
	// method takes two arguments accelerometer data array with normalized acceleration data
	// and array where to store solved points // the double[][][] p array can be variable size
	public static float vertik[] = {0.0f, 0.0f, 12.0f}; // vector that represents accelerometer vector
	public static final float HOR = 8.0f; // horizontal distance between accelerometers
	public static final float P0[] = {0.0f, 0.0f, 0.0f}; // reference accelerometer point
	public static final float A[] = {0.0f, 0.0f, 1.0f};
	public static final float P00[] = {-HOR, 0.0f , 0.0f}; // initial vector positions
	public static final float P02[] = {HOR, 0.0f, 0.0f};
	public static final float P03[] = {2*HOR, 0.0f, 0.0f};
	
	// method that calculates segment point coordinates from normalize acc data
	//arguments :accelerometerData[][] normalized acc data array
	//p[][][] resulting segment point coordinates
	//float accVerticDistance vertical distance between acc segments
	public static void solveAccelerometerPosition(float[][] accelerometerData, float[][][] p, float accVerticDistance){
		vertik[2]=accVerticDistance;
		int sizeColumns=p[0].length; // number of acc grid columns
		int sizeRows=p.length; // number of acc grid rows
		float n[] = new float[3]; // rotation axes normal vector
		float q[] = new float[4]; //quaternion
		//solving n by computing cross product 
		crossProduct(accelerometerData[11],A,n);
		// getting rotation angle
		//double fi = Math.acos(1*accelerometerData[11][2]);// sakrit
		float fi = (float)Math.acos(dotProduct(accelerometerData[11],A));
		//getting quaternions
		quaternion(n, fi,q);
		
		//initial points for each accelermoeter column 
		//first column
		float tempVector[];
		tempVector=new float[3]; // temporary vector 
		quatRotate(q,  P00,tempVector);
		p[0][0][0]=P0[0]+tempVector[0];
		p[0][0][1]=P0[1]+tempVector[1];
		p[0][0][2]=P0[2]+tempVector[2];
		//second column
		p[0][1][0]=P0[0];
		p[0][1][1]=P0[1];
		p[0][1][2]=P0[2];
		// third column
		tempVector=new float[3]; // temporary vector
		quatRotate(q,  P02,tempVector);
		p[0][2][0]=P0[0]+tempVector[0];
		p[0][2][1]=P0[1]+tempVector[1];
		p[0][2][2]=P0[2]+tempVector[2];
		//fourth column
		tempVector=new float[3]; // temporary vector
		quatRotate(q,  P03,tempVector); //rotating vector
		p[0][3][0]=P0[0]+tempVector[0];
		p[0][3][1]=P0[1]+tempVector[1];
		p[0][3][2]=P0[2]+tempVector[2];
		// solve all the remaining points
		for(int i = 1;i<=sizeColumns;i++){
			for(int k = 2;k<=sizeRows;k++){
				int index = 17-((i-1)*4+k-1);
				crossProduct(accelerometerData[index-1],A,n);
				fi = (float)Math.acos(dotProduct(accelerometerData[index-1],A));
				// getting quaternions
				quaternion(n, fi, q);
				tempVector = new float[3];
				quatRotate(q, vertik, tempVector);
				p[k-1][i-1][0]=p[k-2][i-1][0]+tempVector[0]; 	//forming the result
				p[k-1][i-1][1]=p[k-2][i-1][1]+tempVector[1];	//forming the result
				p[k-1][i-1][2]=p[k-2][i-1][2]+tempVector[2];	//forming the result	
			}
		}	
	}
    /**
     * Method that calculate cross product of two vectors v1 v2
     * @param v1 first input vector float[3] array
     * @param v2 second input vector float[3] array
     * @param cross cross product vector float[3] array
     */
	public static void crossProduct(float[] v1, float[] v2, float[] cross){
		cross[0]=v1[1]*v2[2]-v1[2]*v2[1];
		cross[1]=v1[2]*v2[0]-v1[0]*v2[2];
		cross[2]=v1[0]*v2[1]-v1[1]*v2[0];
	}
	/**
	 * Method that calculates dot product of two vectors v1 v2
	 * @param v1  input vector float[n] array
	 * @param v2  input vector float[n] array
	 * @return return dot product of type float
	 */
	public static float dotProduct(float[] v1, float[] v2){
		if(v1.length!=v2.length){
			return 0;
		}
		float sum=0;
		for(int i=0; i<v1.length; i++){
			sum+=v1[i]*v2[i];
		}
		return sum;
	}
	/**
	 * Method that calculates quaternion from given rotation axis and angle
	 * @param n  normalised rotation axis vector float[3] array
	 * @param fi  rotation angle in radians
	 * @param quaternion output quaternion array - float[4]
	 */
	public static void quaternion(float[] n, float fi, float[] quaternion){
		quaternion[0]=(float)Math.cos(fi/2);
		quaternion[1]=n[0]*(float)Math.sin(fi/2);
		quaternion[2]=n[1]*(float)Math.sin(fi/2);
		quaternion[3]=n[2]*(float)Math.sin(fi/2);
	}
	/**
	 * Method rotates vector by quaternion
	 * @param q - input quaternion float[4] array
	 * @param v - input vector float[3] to be rotated
	 * @param result - rotated vector float[3]
	 */
	public static void quatRotate(float[] q, float[] v, float[] result){
		float qrot[][]= new float[3][3]; //rotation matrix
		getQuaternionRotationMatrix(q, qrot); // get quternion rotation matrix from quaternion and store in qrot
		//
		multiplyMatrix(qrot, v, result); // multiply vector by quaterion rotation matrix
		// 
		}
	/**
	 * function that constructs rotation matrix from quaternion
	 * @param qin input quaternion float[4] array
	 * @param qr 3x3 array for output rotation matrix float[3][3]
	 */
	public static void getQuaternionRotationMatrix(float[] qin, float[][] qr){
		float[] q = new float[4];
		float norm=(float)Math.sqrt(Math.pow(qin[0], 2)+Math.pow(qin[1], 2)+Math.pow(qin[2], 2)+Math.pow(qin[3], 2)); // quaternion norm
		for(int i=0;i<=3;i++){ // normalizing quaternion
			q[i]=qin[i]/norm;
		}
		qr[0][0]=(float)Math.pow(q[0], 2)+(float)Math.pow(q[1], 2)-(float)Math.pow(q[2], 2)-(float)Math.pow(q[3], 2);
		qr[0][1]=2*(q[1]*q[2]-q[0]*q[3]);
		qr[0][2]=2*(q[1]*q[3]+q[0]*q[2]);
		qr[1][0]=2*(q[1]*q[2]+q[0]*q[3]);
		qr[1][1]=(float)Math.pow(q[0], 2)-(float)Math.pow(q[1], 2)+(float)Math.pow(q[2], 2)-(float)Math.pow(q[3], 2);
		qr[1][2]=2*(q[2]*q[3]-q[0]*q[1]);
		qr[2][0]=2*(q[1]*q[3]-q[0]*q[2]);
		qr[2][1]=2*(q[2]*q[3]+q[0]*q[1]);
		qr[2][2]=(float)Math.pow(q[0], 2)-(float)Math.pow(q[1], 2)-(float)Math.pow(q[2], 2)+(float)Math.pow(q[3], 2);		
	}
	
	/**
	 * method for multiplying 3x3 matrix with vector.
	 * @param qr input matrix 3x3 float array
	 * @param v input vector float[3]
	 * @param result output vector float[3]
	 */
	public static void multiplyMatrix(float qr[][], float v[], float result[]){
		result[0]=qr[0][0]*v[0]+qr[0][1]*v[1]+qr[0][2]*v[2];
		result[1]=qr[1][0]*v[0]+qr[1][1]*v[1]+qr[1][2]*v[2];
		result[2]=qr[2][0]*v[0]+qr[2][1]*v[1]+qr[2][2]*v[2];
	}
	
	//method that calculates average distance between two accelerometer grid states
	//returns double value
	//two arguments state1[][][] and state2[][][] are arguments, contiining point coordinates for
	//particular state
	public static float getAverageDistance(float[][][] state1 ,float[][][] state2){
		int sizeColumns = state1[0].length;
		int sizeRows = state1.length;
		float distance=0;
		float deltaX_s=0; // saving temporary deltaX squared
		float deltaY_s=0; // saving temporary deltaY squared
		float deltaZ_s=0; //// saving temporary deltaZ squared
		for (int i=0; i<sizeRows;i++){
			for(int k=0;k<sizeColumns;k++){
				deltaX_s=(float)Math.pow(state1[i][k][0]-state2[i][k][0],2);
				deltaY_s=(float)Math.pow(state1[i][k][1]-state2[i][k][1],2);
				deltaZ_s=(float)Math.pow(state1[i][k][2]-state2[i][k][2],2);
				distance=(float)(distance+Math.sqrt(deltaX_s+deltaY_s+deltaZ_s));
			}
		}
		return distance/(sizeRows*sizeColumns); //returning the result
	}
	// returns array of distances between states
	public static float[][] getDistances(float [][] distances, float [][][] state1, float[][][] state2){
		int sizeColumns = state1[0].length;
		int sizeRows = state1.length;
		float deltaX_s=0; // saving temporary deltaX squared
		float deltaY_s=0; // saving temporary deltaY squared
		float deltaZ_s=0; //// saving temporary deltaZ squared
		
		for (int i=0; i<sizeRows;i++){
			for(int k=0;k<sizeColumns;k++){
				deltaX_s=(float)Math.pow(state1[i][k][0]-state2[i][k][0],2);
				deltaY_s=(float)Math.pow(state1[i][k][1]-state2[i][k][1],2);
				deltaZ_s=(float)Math.pow(state1[i][k][2]-state2[i][k][2],2);
				distances[i][k]=(float)Math.sqrt(deltaX_s+deltaY_s+deltaZ_s);
			}
		}
		return distances;
	}
	// finds max distance from distances array
	public float findMaxDistance(float[][] distances){
		int sizeColumns = distances[0].length;
		int sizeRows = distances.length;
		float maxDistance=0;
		for(int i=0;i<sizeRows;i++){
			for(int j=0;j<sizeColumns;j++){
				if(distances[i][j]>maxDistance){
					maxDistance=distances[i][j];
				}
			}
		}
		return maxDistance;
	}
	//method calculates averaged disance between to acc grid sates in sagital plane (xy)
	// arguments state1[][][] and state2[][][] acc grid points.
	public static float getAverageDistanceSagitalPlane(float[][][] state1, float[][][] state2){
		int sizeColumns = state1[0].length;
		int sizeRows = state1.length;
		float distance=0;
		float deltaY_s=0; // temoperary deltaY squared
		float deltaZ_s=0; // temporary deltaZ squared
		for(int i=0; i<sizeRows;i++){
			for(int j=0; j<sizeColumns;j++){
			    deltaY_s=(float)pow(state1[i][j][1]-state2[i][j][1],2);
			    deltaZ_s=(float)pow(state1[i][j][2]-state2[i][j][2],2);
			    distance=(float)(distance+sqrt(deltaY_s+deltaZ_s));
			}
		}
		return distance/(sizeRows*sizeColumns);
	}
	public static float getAverageDistanceCoronalPlane(float[][][] state1, float state2[][][]){
		int sizeColumns = state1[0]. length;
		int sizeRows = state1.length;
		float distance=0;
		float deltaX_s=0;
		float deltaZ_s=0;
		for( int i=0;i<sizeRows;i++){
			for(int j=0; j<sizeColumns;j++){
				deltaX_s=(float)pow(state1[i][j][0]-state2[i][j][0],2);
			    deltaZ_s=(float)pow(state1[i][j][2]-state2[i][j][2],2);
			    distance=(float)(distance+sqrt(deltaX_s+deltaZ_s));
			}
		}
		return distance/(sizeRows*sizeColumns);
	}
	//method that rotates whole point representation of acc grid by quaternion.
	// accPoints[] array of calculated points, q[] quaternion, rotatedAccPoints[] result rotated Acc points
	public static void rotateGrid(float[][][] accPoints, float[] q, float[][][] rotatedAccPoints){
		int sizeColumns = accPoints[0].length;
		int sizeRows = accPoints.length;
		for(int i=0; i<sizeRows; i++){
			for(int j=0;j<sizeColumns;j++){
				quatRotate(q, accPoints[i][j], rotatedAccPoints[i][j]);
			}
		}
	}
	// compensate tilt for refference Points
	public static void compensateTilt(float[][][] accRefPoints, float[][][] accCurrentPoints, float[][][] accRefPointsCompensated){
		float[] n = new float[3]; // rotation axis
		float[] q = new float[4]; // rotation quaternion
		float fi = (float)Math.acos(dotProduct(accRefPoints[1][1],accCurrentPoints[1][1])/(getVectorLength(accRefPoints[1][1])*getVectorLength(accCurrentPoints[1][1])));
		crossProduct(accRefPoints[1][1],accCurrentPoints[1][1],n);
		normalizeVector(n);
		quaternion(n, fi, q);
		rotateGrid(accRefPoints, q, accRefPointsCompensated);
	}
	/**
	 * method returning magnitude of vector
	 * @param vector input vector float[3]
	 * @return magnitude magnitude of input vector of type float
	 */
	public static float getVectorLength(float[] vector){
		return (float)Math.sqrt(Math.pow(vector[0], 2)+Math.pow(vector[1], 2)+Math.pow(vector[2], 2));
	}

	/**
	 * normates input vector
	 * @param vector input vector (after function call this vector is changed)
	 */
	public static void normalizeVector(float[] vector){
		float norm = (float)Math.sqrt(Math.pow(vector[0], 2)+Math.pow(vector[1],2)+Math.pow(vector[2], 2));
		for(int i = 0; i<vector.length;i++){
			float temp=vector[i]/norm;
			vector[i]=temp;
		}
	}

	/**
	 * Mehod returns absolute value of vector lements
	 * @param vector with absolute value of elements
	 */
	public static void absVector(float[] vector){
		for(int i = 0; i<vector.length; i++){
			vector[i]=Math.abs(vector[i]);
		}
	}
	
	/**
	 * method that return quaternion between two vectors
	 * @param vector1 first input vector
	 * @param vector2 second input vector
	 * @param quaternion resulting quaterion
	 */
	public static void quaternionBetweenVectors(float[] vector1, float[]vector2, float[] quaternion){
		float n[] = new float[3]; //rotation axis
		float fi;// rotation angle
		crossProduct(vector1, vector2, n);
		normalizeVector(n);
		fi=(float)Math.acos(dotProduct(vector1, vector2));
		quaternion(n, fi, quaternion);
		normalizeQuaternion(quaternion);
	}
	/**
	 * normates input quaterion
	 * @param quaternion input quaternion. after function call this quaternion is changed
	 */
	public static void normalizeQuaternion(float quaternion[]){
		float length=(float)Math.sqrt(Math.pow(quaternion[0], 2)+Math.pow(quaternion[1], 2)+Math.pow(quaternion[2], 2)+Math.pow(quaternion[3], 2));
		float temp;
		for(int i = 0; i<=3; i++){
			temp=quaternion[i]/length;
			quaternion[i]=temp;
		}
	}
	//method that calculates angles between adjacent accelerometers 
	//input argument accData[] normalized accelerometer data array
	//accQuaternion[][][] output array for quaternions
	public static void solveAccGridQuaternions(float[][] accData, float accQuaternions[][][]){
		float vec1[] = new float[3]; // lower acc
		float vec2[] = new float[3]; // upper acc
		float tempQuat[] = new float[4]; // temp quaternin to get acc segment rotation
		for(int j=0; j<accQuaternions[0].length; j++ ){
			for(int i=0; i<accQuaternions.length; i++){
				quaternionBetweenVectors(accData[15-(4*j+i)], A, tempQuat);
				quatRotate(tempQuat, A, vec1); // get vector that represents lower acc
				quaternionBetweenVectors(accData[15-(4*j+i+1)], A, tempQuat);
				quatRotate(tempQuat, A, vec2); // get vector for upper acc
				quaternionBetweenVectors(vec1, vec2, accQuaternions[i][j]);
				
			}
		}
	}
	// returns accelerometer rotatoin angle in sagital plane around x axis
	// accData[] normalized accelerometer data array
	public static float getSagitalPlaneAngle(float accData[]){
		return (float)-Math.atan2(accData[1],Math.sqrt(Math.pow(accData[0], 2)+Math.pow(accData[2], 2)));
	}
	//return accelerometer rotation angle in coronal plane around y axis
	// accData[] normalized accelerometer data array
	public static float getCoronalPlaneAngle(float accData[]){
		return (float)-Math.atan2(accData[0],Math.sqrt(Math.pow(accData[1], 2)+Math.pow(accData[2], 2)));
	}
	
	// solves angles between adjacent accelerometers in colum
	// accData[] accelerometer data array
	//accAngles[][] resulting angles
	public static void getAnglesBetweenAccVectors(float accData[][], float[][][] angles){
		int rows = angles.length;
		int columns = angles[0].length;
		for (int j=0; j<columns;j++){
			for(int i=0; i<rows;i++){
				angles[i][j][0] = (float)(Math.PI-getCoronalPlaneAngle(accData[15-(4*j+i)])-getCoronalPlaneAngle(accData[15-(4*j+i+1)])); // coronal plane angle difference
				angles[i][j][1] = (float)(Math.PI-getSagitalPlaneAngle(accData[15-(4*j+i)])-getSagitalPlaneAngle(accData[15-(4*j+i+1)])); //sagital plane angle difference
			}
		}	
	}
	public static boolean isAngleOverThreshold(float[][][] accSavedAngles, float [][][] accCurrentAngles, float threshold){
		int rows = accSavedAngles.length;
		int columns = accSavedAngles[0].length;
		float deltaAngleCoronal;
		float deltaAngleSagital;
		boolean isOverThreshold = false;
		for(int j=0; j<columns; j++){
			for(int i=0; i<rows; i++){
				deltaAngleCoronal=Math.abs(accSavedAngles[i][j][0]-accCurrentAngles[i][j][0]);
				deltaAngleSagital=Math.abs(accSavedAngles[i][j][1]-accCurrentAngles[i][j][1]);
				if((deltaAngleCoronal >= threshold)||(deltaAngleSagital>= threshold)){
					isOverThreshold = true;
				}else {
					isOverThreshold = false;
				}
			}
		}
		return isOverThreshold;
	}
	
	// get maximums angle difference for sagital and coronal plane angles
	public static float getMaximumAngleDifference (float[][][] savedAngles, float[][][] currentAngles){
		int sizeRows=savedAngles.length;
		int sizeCols=savedAngles[0].length;
		float maximumAngleDifference=0;
		float angleDifferenceTemp;
		for(int i = 0; i<sizeCols;i++){
			for(int j=0; j<sizeRows;j++){
				if(Math.abs(savedAngles[j][i][0]-currentAngles[j][i][0])>Math.abs(currentAngles[j][i][1]-currentAngles[j][i][1])){
					angleDifferenceTemp=Math.abs(savedAngles[j][i][0]-currentAngles[j][i][0]);
				} else{
					angleDifferenceTemp=Math.abs(currentAngles[j][i][1]-currentAngles[j][i][1]);
				}
				if(angleDifferenceTemp>maximumAngleDifference){
					maximumAngleDifference=angleDifferenceTemp;
				}
			}
		}
		return maximumAngleDifference;
	}	
	// returns maximum distance between two point arrays
	public static float getMaxDistance(float[][][] savedPoints, float[][][] currentPoints){
		float maxDistance=0;
		float currentDistance;
		int rows = savedPoints.length;
		int cols = savedPoints[0].length;
		for(int i=0;i<rows;i++){
			for(int j=0;j<cols;j++){
				currentDistance=(float)sqrt(pow(savedPoints[i][j][0]-currentPoints[i][j][0],2)+
									 pow(savedPoints[i][j][1]-currentPoints[i][j][1],2)+
									 pow(savedPoints[i][j][2]-currentPoints[i][j][2],2));
				if(currentDistance>maxDistance){
					maxDistance=currentDistance;
				}
			}
		}
		return maxDistance;
	}
	

	public static float getMaxDistanceSagital(float[][][] savedPoints, float[][][] currentPoints){
		float maxSagitalDistance=0;
		float currentSagitalDistance;
		int rows = savedPoints.length;
		int cols = savedPoints[0].length;
		for(int i=0;i<rows;i++){
			for(int j=0;j<cols;j++){
				currentSagitalDistance=(float)sqrt(pow(savedPoints[i][j][1]-currentPoints[i][j][1],2)+
									 		pow(savedPoints[i][j][2]-currentPoints[i][j][2],2));
				if(currentSagitalDistance>maxSagitalDistance){
					maxSagitalDistance=currentSagitalDistance;
				}
			}
		}
		return maxSagitalDistance;
	}
	
	public static float getMaxDistanceCoronal(float[][][] savedPoints, float[][][] currentPoints){
		float maxCoronalDistance=0;
		float currentCoronalDistance;
		int rows = savedPoints.length;
		int cols = savedPoints[0].length;
		for(int i=0;i<rows;i++){
			for(int j=0;j<cols;j++){
				currentCoronalDistance=(float)sqrt(pow(savedPoints[i][j][0]-currentPoints[i][j][0],2)+
									 pow(savedPoints[i][j][1]-currentPoints[i][j][1],2)+
									 pow(savedPoints[i][j][2]-currentPoints[i][j][2],2));
				if(currentCoronalDistance>maxCoronalDistance){
					maxCoronalDistance=currentCoronalDistance;
				}
			}
		}
		return maxCoronalDistance;
	}
	public static float getMaxDistanceFromDistances(float distances[][]){
		float maxDistance = 0;
		for(int i=0;i<distances.length;i++){
			for(int j=0;j<distances[0].length;j++){
				if(distances[i][j]>maxDistance){
					maxDistance=distances[i][j];
				}
			}
		}
		return maxDistance;
	}

	public static float getMaxDistanceFromDistances(Vector<Vector <Float>> distances ){
		float maxDistance=0;
		for(int i=0; i<distances.size(); i++){
			for(int j=0; j<distances.get(0).size(); j++){
				maxDistance=Math.max(distances.get(i).get(j).floatValue(), maxDistance);
			}
		}
		return maxDistance;
	}
	/**
	 * returns maximum distance of sensors, ignoring upper 4 sensors  for both sides
	 * @param distances input array of distances
	 * @return maximum distance
	 */
	public static float getMaxDistanceFromDistancesReduced(float distances[][]){
		float maxDistance = 0;
		for(int i=0;i<distances.length;i++){
			for(int j=0;j<distances[0].length;j++){
				if(!(((i>=(distances.length-4))&&(j==0))||((i>=(distances.length-4))&&(j==(distances[0].length-1))))){
					if(distances[i][j]>maxDistance){
						maxDistance=distances[i][j];
					}
			    }
			}
		}
		return maxDistance;
	}
	/**
	 * method that returns row and column index for particular accelerometer, identified by it's index
	 * @param index index of accelerometer
	 * @param ROWS number of rows in sensor grid
	 * @param COLS number of columns in sensor grid
	 * @return indexes[] returns array of indexes index[0] - row index, index[1] column index
	 */
	
	public static int[] getIndexes(int index, int ROWS, int COLS){
		int indexes[]={0, 0};  // defines indexes array
//		for(int c=0;c<=COLS-2;c++){
//			if ((int)(index/(ROWS*COLS-ROWS*(c+1)))>0){
//				if((c%2==0)^(COLS%2==0)){
//					indexes[0]=index-(ROWS*COLS-(c+1)*ROWS);// row
//					indexes[1]=c;							// col
//					return indexes;
//				}else{
//					indexes[0]=(ROWS-1)-(index-(ROWS*COLS-(c+1)*ROWS));
//					indexes[1]=c;
//					return indexes;
//				}
//			}
//		}
//		indexes[1]=COLS-1;
//		indexes[0]=index;
		indexes[1]=index/ROWS;
		if((indexes[1]%2)==0){
			indexes[0]=index % ROWS;
		} else{
			indexes[0]=ROWS-1-index%ROWS;
		}
		return indexes;	
	}

	/** methodreturns sensor identifier by taking sensors location in grid.
	 * @param row - sensor row location in the grid
	 * @param col - sensor column location in the grid
	 * @param ROWS - total number of rows of sensor grid
	 * @param COLS - total nomber of columns of sensor grid
	 * @param startLeft - flag that shows if sensors in grid are mounted with first sensor (index 0)
	 *        				mounted on the left side. if truer first sensor is on lefts side if false
	 *        				sensor is on the right side
	 *@return sensor identifier as integer
	 *
	 */
	public static int getIndex(int row, int col, int ROWS, int COLS, boolean startLeft){
		int index=0;
		if(!startLeft){
			if(col%2==0){ // if upgoing column
				index = col*ROWS+row;
			} else{
				index = col*ROWS+(ROWS-row-1);
			}
		}else{
			int firstDown =(COLS+1)%2;
			if((col+firstDown)%2==0){
				index=ROWS*COLS-(col*ROWS+row)-1;
			} else{
				index=ROWS*COLS-(col*ROWS+(ROWS-row-1))-1;
			}
		}
		return index;
	}
	
	// Quaternion operations
	/**
	 * method for calculating conjugate of quaternion
	 * @param qin input quaternion float[4] array
	 * @param qout conjugate of input quaterinon float[4] array
	 */
	public void quatConj(float[] qin, float[] qout){
		qout[0]=qin[0];
		qout[1]=-qin[1];
		qout[2]=-qin[2];
		qout[3]=-qin[3];
	}
	/**
	 * method for finding inverse quaternion
	 * @param qin input quaternion float[4] array
	 * @param qout inverse of the input quaternion float[4] quaternion
	 */
	public void quatInv(float[] qin, float[] qout){
		float sq=(float)(pow(qin[0],2)+pow(qin[1],2)+pow(qin[2],2)+pow(qin[3],2));
		float[] conj=new float[4]; // array for storing temporary quaternion
		quatConj(qin, conj); // calculate conjugate quaternion
		for(int i=0;i<qin.length;i++){
			qout[i]=conj[i]/sq;
		}
	}
	/**
	 * Method for quaternion multiplication
	 * @param p first input quaternion
	 * @param q second input quaternion
	 * @param qout output quaternion (product of two input quaternions) 
	 */
	public void quatMult(float[] p, float[] q, float[] qout){
		qout[0]=p[0]*q[0]-(p[1]*q[1]+p[2]*q[2]+p[3]*q[3]);
		qout[1]=p[0]*q[1]+q[0]*p[1]+p[2]*q[3]-p[3]*q[2];
		qout[2]=p[0]*q[2]+q[0]*p[2]+p[3]*q[1]-p[1]*q[3];
		qout[3]=p[0]*q[3]+q[0]*p[3]+p[1]*q[2]-p[2]*q[1];
	}
	
	/**
	 * Function from matrix and matrix transpose multiplication (A*B'). Note B is transposed before multiplication
	 * @param A 3x3 matrix as float[][] - A
	 * @param B 3x3 matrix as float[][] - B
	 * @return 
	 */
	public static float[][] multMatMatT(float[][] A, float[][] B){
		float[][] res = new float[3][3];
		for(int i=0; i<3; i++){
			float[] vec1 = new float[3];
			for(int z=0; z<3; z++){
				vec1[z]=A[i][z];
			}
			for(int j=0; j<3; j++){
				float[] vec2 = new float[3];
				for(int z=0; z<3; z++){
					vec2[z]=B[j][z];
				}
				res[i][j]=dotProduct(vec1, vec2);
			}
		}
		return res;
	};
	
	/** Translates vector by vector. input vector is changes by vector
	 * inputVec vector that are to be translated
	 * offset - array of length 3 representing offset
	 * translated - translated vector, must be initialised before function call
	 */
	public static void translateVec(float inputVec[], float[] offset, float[] translated){
		translated[0]=inputVec[0]+offset[0];
		translated[1]=inputVec[1]+offset[1];
		translated[2]=inputVec[2]+offset[2];
	}
	/**
	 * Function for rotation matrix estimation with TRIAD algorithm, that aligns
	 * sensor local reference frame to global reference frame.
	 * @param acc_data - accelerometer data array of size 3 with normalised accelerometer data
	 * @param magn_data  - magnetometer data array of size 3 with normalised magnetometer data
	 * return float[][] - result array  of size [3][3] must be initialised before function call*/
	public static float[][] getRotationTRIAD(float[] acc_data, float[] magn_data){
		float[][] result = new float[3][3];
//		float[] magn_data={1, 0, 0};
		if(acc_data.length==3 && magn_data.length==3){
			float[] Eg = {0, 0, 1};
			float[] Em = {0.4472f, 0, -0.8944f};
			float[] s2 = new float[3]; 
			crossProduct(Eg, Em, s2);
			normalizeVector(s2);
			float[] s3 = new float[3];
			crossProduct(Eg, s2, s3);
			
			float[] r2 = new float[3];
			crossProduct(acc_data, magn_data, r2);
			normalizeVector(r2);
			float[] r3 = new float[3];
			crossProduct(acc_data, r2, r3);
			
			float[][] Mmeas = new float[3][3];
			float[][] Mref = new float[3][3];
			
			// filling Mmeas and Mref matrices
			for(int i=0; i<Mmeas.length; i++){
				Mmeas[i][0]=acc_data[i];
				Mmeas[i][1]=r2[i];
				Mmeas[i][2]=r3[i];
				
				Mref[i][0]=Eg[i];
				Mref[i][1]=s2[i];
				Mref[i][2]=s3[i];
				
			}
			result = multMatMatT(Mref, Mmeas);
		}
		return result;
	}
	
	public static void transpose(float[][] mat){
		float[][] res = new float[mat.length][mat[0].length];
		for(int i=0; i<mat.length; i++){
			for(int j=0; j<mat[0].length; j++){
				res[j][i]=mat[i][j];
			}
		}
		for(int i=0; i<mat.length; i++){
			for(int j=0; j<mat[0].length; j++){
				mat[i][j]=res[i][j];
			}
		}
	}
	/**
	 * Method performs Kalman filtering on specified data

	 * @param outputFilteredData in this array output result will be stored. Previous value is used to compute current value
	 * @param P_xyz probabilities array of length array which is changed after each iteration
	 * @param Q Kalman filter Q parameter

	 */
	public static void kalmanFilter(float[] inpuRawData, float[] outputFilteredData, float[] P_xyz, float Q, float delta){
		float Pn;
		float K;
		for(int i=0; i<3; i++){
			Pn=P_xyz[i]+Q;
			K=Pn/(Pn+delta);
			outputFilteredData[i]=outputFilteredData[i]+K*(inpuRawData[i]-outputFilteredData[i]);
			P_xyz[i]=(1-K)*Pn;
		}
	}
	
}
