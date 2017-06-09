/*
 * Filename:        CTransformation.cs
 * Author:          Qinbing Fu
 * Date:            Jan 2017
 * Description:     This code is rewritted through [1], [2] coding in C++. Transforms detected ellipse position and dimensions to arbitrary 3D or 2D coordinates frame. The method is described in Chapter 4 of the article [1].
 * Licence:         If you use this class, please cite [1], [2].
 * References:  [1] Krajnik, Nitsche et al.: A practical multirobot localization system. Journal of Intelligent and Robotic Systems, 2014.
 * References:  [2] Peter Lightbody, et al.: A Versatile High-Performance Visual Fiducial Marker Detection System with Scalable Identity Encoding. The 32nd ACM Symposium on Applied Computing, SAC 2017.
 */

using System;
using System.Collections.Generic;

namespace WhyConID
{
    //which transform to use
    public enum ETransformType
    {
        TRANSFORM_NONE,			//camera-centric
        TRANSFORM_2D,			//3D->2D homography 
        TRANSFORM_3D,			//3D user-defined - linear combination of four translation/rotation transforms 
        TRANSFORM_4D,			//3D user-defined - full 4x3 matrix
        TRANSFORM_INV,			//for testing purposes
        TRANSFORM_NUMBER
    };

    public struct STrackedObject
    {
        public float x, y, z, d;			//position and distance from the camera
        public float pitch, roll, yaw;		//angles - precision is unknown
        public float roundness;		        //segment roundness as calculated by Eq. 5 of [1]
        public float bwratio;			    //black/white area ratio
        public float error;			        //measured error
        public float esterror;			    //predicted error
        public int ID;				        //ID - not used here
    }

    //rotation/transformation model of the 3D transformation
    public struct S3DTransform
    {
        public float[,] simlar;            //rotation description
        public STrackedObject orig;        //translation vector
    }
    public class CTransformation
    {
        #region FIELDS
        private const byte NMAX = 11;
        private const byte MMAX = 6;
        private const double MACH_EPS = 1e-20;
        private const byte MATSIZE = 3;

        public ETransformType transformType;
        private float[] xArray;
		private float[] yArray;
		private float[] gArrayX;
		private float[] gArrayY;
		private int[] pArray;
        private float[] hom;
		private float[] trf4D;
		private float gDimX,gDimY;
		private S3DTransform[] D3transform;
		private int width,height;
		private bool fullUnbarrel;
		//private bool unbarrelInitialized;
		private float trackedObjectDiameter;
		private float[] kc;
		private float[] kcerr;
		private float[] fcerr;
		private float[] fc;
		private float[] cc;
		//private float error2D;
		private STrackedObject[] c2D;
        #endregion

        #region METHODS
        //initialization: width and height of the image, diameter of the pattern, 'unbarrel the entire image' flag
        /// <summary>
        /// Default Constructor
        /// </summary>
        public CTransformation()
        { }

        /// <summary>
        /// Parameterized Constructor
        /// </summary>
        /// <param name="widthi"></param>
        /// <param name="heighti"></param>
        /// <param name="diam"></param>
        /// <param name="fullUnbarreli"></param>
        public CTransformation(int widthi, int heighti, float diam, bool fullUnbarreli = false)
        {
            hom = new float[9];
            trf4D = new float[16];
            D3transform = new S3DTransform[4];
            kc = new float[6];
            kc[0] = 1.0f;
            kcerr = new float[6];
            fcerr = new float[2];
            fc = new float[2];
            cc = new float[2];
            c2D = new STrackedObject[4];

            transformType = ETransformType.TRANSFORM_NONE;
	        fullUnbarrel = fullUnbarreli;
	        width = widthi;
	        height = heighti;
	        trackedObjectDiameter = diam;
	        //unbarrelInitialized = false;

            if (fullUnbarrel)
            {
                //unbarrelInitialized = true;
                float ix, iy;
                float gx, gy, ux, uy;
                xArray = new float[width * height];
                yArray = new float[width * height];
                gArrayX = new float[width * height];
                gArrayY = new float[width * height];
                pArray = new int[width * height * 4];

                for (int x = 0; x < width; x++)
                {
                    for (int y = 0; y < height; y++)
                    {
                        xArray[y * width + x] = barrelX(x, y);
                        yArray[y * width + x] = barrelY(x, y);
                        if (xArray[y * width + x] < 0 || xArray[y * width + x] > (width - 1) || yArray[y * width + x] < 0 || yArray[y * width + x] > (height - 1))
                        {
                            xArray[y * width + x] = 0;
                            yArray[y * width + x] = 0;
                        }
                        ux = (float)Math.Truncate(xArray[y * width + x]);
                        uy = (float)Math.Truncate(yArray[y * width + x]);
                        gx = xArray[y * width + x] - ux;
                        gy = yArray[y * width + x] - uy;
                        ix = ux;
                        iy = uy;
                        pArray[y * width + x] = (int)(width * iy + ix);
                        gArrayX[y * width + x] = gx;
                        gArrayY[y * width + x] = gy;
                    }
                }
            }
        }
        public float barrelX(float x, float y)
        {
            x = (x-cc[0])/fc[0];
	        y = (y-cc[1])/fc[1];
	        float cx,dx;
	        float r = x*x+y*y;
	        dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
	        cx = (1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r)*x+dx;
	        cx = cx*fc[0]+cc[0];
	        return cx;
        }

        public float barrelY(float x, float y)
        {
            x = (x-cc[0])/fc[0];
	        y = (y-cc[1])/fc[1];
	        float cy,dy;
	        float r = x*x+y*y;
	        dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
	        cy = (1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r)*y+dy;
	        cy = cy*fc[1]+cc[1];
	        return cy;
        }

        public float unbarrelX(float x, float y)
        {
            if (fullUnbarrel)return x;
	        float ix,iy,dx,dy,r,rad;
	        ix = x = (x-cc[0])/fc[0];
	        iy = y = (y-cc[1])/fc[1];
	        for (int i= 0;i<5;i++){
		        r = x*x+y*y;
		        dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
		        dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
		        rad = 1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r;
		        x = (ix-dx)/rad;
		        y = (iy-dy)/rad;
	        }
	        return (x*fc[0]+cc[0]);
        }

        public float unbarrelY(float x, float y)
        {
            if (fullUnbarrel) return y;
	        float ix,iy,dx,dy,r,rad;
	        ix = x = (x-cc[0])/fc[0];
	        iy = y = (y-cc[1])/fc[1];
	        for (int i= 0;i<5;i++){
		        r = x*x+y*y;
		        dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
		        dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
		        rad = 1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r;
		        x = (ix-dx)/rad;
		        y = (iy-dy)/rad;
	        }
	        return (y*fc[1]+cc[1]);
        }

        //undistort the entire image
        public void unbarrel(ref byte[] dst, ref byte[] src)
        {
            src[0] = src[1] = src[2] = 255;
	        if (fullUnbarrel){
		        float gx,gy;
		        for (int p = 0;p<width*(height-1);p++){
			        gx = gArrayX[p];
			        gy = gArrayY[p];
			        dst[3*p] = (byte)(src[3*pArray[p]]*(1-gx)*(1-gy)+src[3*pArray[p]+3]*gx*(1-gy)+src[3*pArray[p]+width*3]*(1-gx)*gy+src[3*pArray[p]+(width+1)*3]*gx*gy); 
			        dst[3*p+1] = (byte)(src[3*pArray[p]+1]*(1-gx)*(1-gy)+src[3*pArray[p]+3+1]*gx*(1-gy)+src[3*pArray[p]+width*3+1]*(1-gx)*gy+src[3*pArray[p]+(width+1)*3+1]*gx*gy); 
			        dst[3*p+2] = (byte)(src[3*pArray[p]+2]*(1-gx)*(1-gy)+src[3*pArray[p]+3+2]*gx*(1-gy)+src[3*pArray[p]+width*3+2]*(1-gx)*gy+src[3*pArray[p]+(width+1)*3+2]*gx*gy); 
		        }
	        }else{
		        Console.WriteLine("Image unbarrel was not enabled\n");
	        }
        }

        //image to canonical coordinates (unbarrel + focal center and length)
        public float transformX(float xc, float yc)
        {
            return (unbarrelX(xc,yc)-cc[0])/fc[0];
        }
        public float transformY(float xc, float yc)
        {
            return (unbarrelY(xc,yc)-cc[1])/fc[1];
        }
        public void transformXY(ref float ax, ref float ay)
        {
            float x,y,ix,iy,dx,dy,r,rad;
	        ax = ix = x = (ax-cc[0])/fc[0];
	        ay = iy = y = (ay-cc[1])/fc[1];
	        if (fullUnbarrel)return;
	        for (int i= 0;i<5;i++){
		        r = x*x+y*y;
		        dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
		        dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
		        rad = 1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r;
		        x = (ix-dx)/rad;
		        y = (iy-dy)/rad;
	        }
	        ax=x;
	        ay=y;
        }
        public void transformXYerr(ref float ax, ref float ay) 
        {
            float x,y,dx,dy,r,rad;
	        //*ax = x = (*ax-cc[0])/fc[0];
	        //*ay = y = (*ay-cc[1])/fc[1];
	        x = ax;
	        y = ay;
	        if (fullUnbarrel)return;
	        r = x*x+y*y;
	        dx = 2*kcerr[3]*x*y + kcerr[4]*(r + 2*x*x);
	        dy = 2*kcerr[4]*x*y + kcerr[3]*(r + 2*y*y);
	        rad = kcerr[1]*r+kcerr[2]*r*r+kcerr[5]*r*r*r;
	        ax=rad*x+dx;
	        ay=rad*y+dy;
        }

        //calculate marker 3D or 2D coordinates in user-defined coordinate system from the segment description provided by the CCircleDetector class, see 4.1-4.4 of [1]
		public STrackedObject transform(SSegment segment, bool unbarreli)
        {
            float x,y,x1,x2,y1,y2,major,minor,v0,v1;
	        STrackedObject result = new STrackedObject();
	        fullUnbarrel = unbarreli;
	
	        /* transformation to the canonical camera coordinates, see 4.1 of [1]*/
	        x = segment.x;
	        y = segment.y;
	        transformXY(ref x,ref y);
	        //major axis
	        //vertices in image coords
	        x1 = segment.x+segment.v0*segment.m0*2;
	        x2 = segment.x-segment.v0*segment.m0*2;
	        y1 = segment.y+segment.v1*segment.m0*2;
	        y2 = segment.y-segment.v1*segment.m0*2;
	        //vertices in canonical camera coords 
	        transformXY(ref x1,ref y1);
	        transformXY(ref x2,ref y2);
	        //semiaxes length 
	        major = (float)(Math.Sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0);
	        v0 = (float)((x2-x1)/major/2.0);
	        v1 = (float)((y2-y1)/major/2.0);
	        //printf("AAA: %f %f\n",sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1))-sqrt((x-x2)*(x-x2)+(y-y2)*(y-y2)),major);

	        //the minor axis 
	        //vertices in image coords
	        x1 = segment.x+segment.v1*segment.m1*2;
	        x2 = segment.x-segment.v1*segment.m1*2;
	        y1 = segment.y-segment.v0*segment.m1*2;
	        y2 = segment.y+segment.v0*segment.m1*2;
	        //vertices in canonical camera coords 
	        transformXY(ref x1,ref y1);
	        transformXY(ref x2,ref y2);
	        //minor axis length 
	        minor = (float)Math.Sqrt(((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0);
	        //printf("BBB: %f %f\n",sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1))-sqrt((x-x2)*(x-x2)+(y-y2)*(y-y2)),minor);

	        /*construct the ellipse characteristic equation, see 4.2 of [1], prepare coefs for Eq */
	        float a,b,c,d,e,f;
	        a = v0*v0/(major*major)+v1*v1/(minor*minor);
	        b = (float)(v0*v1*(1.0/(major*major) -1.0/(minor*minor)));
	        c = v0*v0/(minor*minor)+v1*v1/(major*major);
	        d = (-x*a-b*y);
	        e = (-y*c-b*x);
	        f = (float)(a*x*x+c*y*y+2*b*x*y-1.0);
	        double[] data ={a,b,d,b,c,e,d,e,f};		//matrix conic coefficients, see 4.2 of [1]

	        /*transformation to camera-centric or user-defined coordinate frames*/
	        //3D->2D homography, see 4.4.2 of [1]
	        if (transformType == ETransformType.TRANSFORM_2D)			
	            {
		            //for debug only
		            result = eigen(data);
		            float dd = (float)Math.Sqrt(result.x*result.x+result.y*result.y+result.z*result.z);
		            result.x = x;
		            result.y = y;
		            result = transform2D(result);
		            result.d = dd;
                    
		            float xerr = x;
		            float yerr = y;
		            transformXYerr(ref xerr,ref yerr);
		            result.esterror = (float)Math.Abs(Math.Sqrt(xerr*xerr+yerr*yerr))*30; 
		            result.yaw = (float)Math.Atan2(segment.v0,segment.v1);
		            result.yaw = segment.angle;
	            }
	        //camera-centric coordinate frame, see 4.3 and 4.4 of [1]
	        if (transformType == ETransformType.TRANSFORM_NONE)
	            {
		            result = eigen(data);
	            }
	        //user-defined 3D coordinate system, see 4.4.1 of [1]
	        if (transformType == ETransformType.TRANSFORM_3D){
		        result = eigen(data);
		        float dd = (float)(Math.Sqrt(result.x*result.x+result.y*result.y+result.z*result.z));
		        result = transform3D(result);
		        result.esterror += (float)(15.0/(segment.m1*4));
		        result.d = dd;
	            }
	        //alternative calculation of 3D->3D transform
	        if (transformType == ETransformType.TRANSFORM_4D){
		        result = eigen(data);
		        float dd = (float)Math.Sqrt(result.x*result.x+result.y*result.y+result.z*result.z);
		        result = transform4D(result);
		        result.d = dd;
	            }
	        //camera centric + error estimate
	        if (transformType == ETransformType.TRANSFORM_INV){
		        result = eigen(data);
		        float dd = (float)Math.Sqrt(result.x*result.x+result.y*result.y+result.z*result.z);
		        result.d = dd;
	            }
	        result.ID = segment.ID;
	        result.yaw = segment.angle;
	        /*result.pitch = acos(fmin(minor/major,1.0))/M_PI*180.0; //TODO
	        result.roll = segment.horizontal; //TODO*/
	        return result;
        }
		/*calculate the pattern 3D position from its ellipse characteristic equation, see 4.3 of [1]*/
		public STrackedObject eigen(double[] data)
        {
            STrackedObject result = new STrackedObject();
	        result.error = 0;
	        double[] d = new double[3];
	        double[,] V = new double[3, 3];
	        double[,] dat = new double[3, 3];
	        for (int i = 0; i < 9; i++) dat[i / 3, i % 3] = data[i];
	        eigen_decomposition(dat, ref V, ref d);

	        //eigenvalues
	        double L1 = d[1];
	        double L2 = d[2];
	        double L3 = d[0];
	        //eigenvectors
	        int V2=2;
	        int V3=0;

	        //detected pattern position
	        float z = (float)(trackedObjectDiameter/Math.Sqrt(-L2*L3)/2.0);
	        float c0 = (float)(Math.Sqrt((L2-L1)/(L2-L3)));
	        float c0x = (float)(c0*V[2, V2]);
	        float c0y = (float)(c0*V[1, V2]);
	        float c0z = (float)(c0*V[2, V2]);
	        float c1 = (float)Math.Sqrt((L1-L3)/(L2-L3));
	        float c1x = (float)(c1*V[0, V3]);
	        float c1y = (float)(c1*V[1, V3]);
	        float c1z = (float)(c1*V[2, V3]);

	        float z0 = (float)(-L3*c0x+L2*c1x);
	        float z1 = (float)(-L3*c0y+L2*c1y);
	        float z2 = (float)(-L3*c0z+L2*c1z);
	        float s1,s2;
	        s1=s2=1;
	        float n0 = +s1*c0x+s2*c1x;
	        float n1 = +s1*c0y+s2*c1y;
	        float n2 = +s1*c0z+s2*c1z;

	        //n0 = -L3*c0x-L2*c1x;
	        //n1 = -L3*c0y-L2*c1y;
	        //n2 = -L3*c0z-L2*c1z;
	        
	        //rotate the vector accordingly
	        if (z2*z < 0){
		        z2 = -z2;
		        z1 = -z1;
		        z0 = -z0;
	        //	 n0 = -n0;
	        //	 n1 = -n1;
	        //	 n2 = -n2;
	        }
	        result.x = z2*z;	
	        result.y = -z0*z;	
	        result.z = -z1*z;
	        result.pitch = n0;//cos(segment.m1/segment.m0)/M_PI*180.0;
	        result.roll = n1;//atan2(segment.v1,segment.v0)/M_PI*180.0;
	        result.yaw = n2;//segment.v1/segment.v0;
	        //result.roll = n2*z;	
	        //result.pitch = -n0*z;	
	        //result.yaw = -n1*z;


	        return result;
        }
		/*establish the user-defined coordinate system from four calibration patterns - see 4.4 of [1]*/
		public int calibrate2D(STrackedObject[] inp,float dimX,float dimY,float robotRadius = 0,float robotHeight =0,float cameraHeight = 1.0f)
        {
            STrackedObject[] r = new STrackedObject[4];
	        STrackedObject[] o = new STrackedObject[4];
	        /*specific to the pheromone system - compensates the fact, that the calibration patterns are displayed in a lower position than the robots
	        assumes that the camera above the field centre*/
	        float iX = dimX/cameraHeight*robotHeight/2;
	        float iY = dimY/cameraHeight*robotHeight/2;

	        //float iX = dimX/(inp[0].x+inp[1].x+inp[2].x+inp[3].x)*4*off;
	        //float iY = dimY/(inp[0].x+inp[1].x+inp[2].x+inp[3].x)*4*off;

	        r[0].x = robotRadius+iX;
	        r[0].y = robotRadius+iY;
	        r[1].x = dimX-robotRadius-iX;
	        r[1].y = robotRadius+iY;
	        r[2].x = robotRadius+iX;
	        r[2].y = dimY-robotRadius-iY;
	        r[3].x = dimX-robotRadius-iX;
	        r[3].y = dimY-robotRadius-iY;
	        for (int i = 0;i<4;i++){
		        o[i].x = -inp[i].y/inp[i].x;
		        o[i].y = -inp[i].z/inp[i].x;
	        }

	        double[,] est = new double[NMAX,NMAX];
            double[,] vec = new double[NMAX,MMAX];
	        double det;
	        for (int i = 0;i<4;i++){
		        est[2*i,0]=-o[i].x;
		        est[2*i,1]=-o[i].y;
		        est[2*i,2]=-1;
		        est[2*i,3]=0;
		        est[2*i,4]=0;
		        est[2*i,5]=0;
		        est[2*i,6]=r[i].x*o[i].x;
		        est[2*i,7]=r[i].x*o[i].y;
		        est[2*i+1,0]=0;
		        est[2*i+1,1]=0;
		        est[2*i+1,2]=0;
		        est[2*i+1,3]=-o[i].x;
		        est[2*i+1,4]=-o[i].y;
		        est[2*i+1,5]=-1;
		        est[2*i+1,6]=r[i].y*o[i].x;
		        est[2*i+1,7]=r[i].y*o[i].y;
		        vec[2*i,0]=-r[i].x;
		        vec[2*i+1,0]=-r[i].y;
	        }
	        MATINV(8,1,ref est,ref vec,out det);
	        for (int i = 0;i<8;i++)  hom[i] = (float)vec[i,0];
	        hom[8] = 1;
	        transformType = ETransformType.TRANSFORM_2D;
	        return 0;
        }
		public int calibrate3D(STrackedObject[] o,float gridDimX,float gridDimY)
        {
            D3transform[0] = calibrate3D(o[0], o[1], o[2], gridDimX, gridDimY);
            D3transform[1] = calibrate3D(o[1], o[0], o[3], gridDimX, gridDimY);
            D3transform[2] = calibrate3D(o[2], o[3], o[0], gridDimX, gridDimY);
            D3transform[3] = calibrate3D(o[3], o[2], o[1], gridDimX, gridDimY);
            gDimX = gridDimX;
            gDimY = gridDimY;
            transformType = ETransformType.TRANSFORM_3D;
            return 0;
        }
		public int calibrate4D(STrackedObject[] o,float gridDimX,float gridDimY)
        {
            //OBSOLETE
            return 0;
        }
		public S3DTransform calibrate3D(STrackedObject o0,STrackedObject o1,STrackedObject o2,float gridDimX,float gridDimY)
        {
            S3DTransform result = new S3DTransform();
	        STrackedObject[] v = new STrackedObject[3];
	        result.orig = o0;

            double[,] m23D = new double[NMAX, NMAX];
            double[,] vec = new double[NMAX, MMAX];
            double det;

	        v[0].x = o1.x-o0.x;
	        v[0].y = o1.y-o0.y;
	        v[0].z = o1.z-o0.z;
	        //v[0] = normalize(v[0]);

	        v[1].x = o2.x-o0.x;
	        v[1].y = o2.y-o0.y;
	        v[1].z = o2.z-o0.z;
	        //v[1] = normalize(v[1]);

	        v[2].x = +v[0].y*v[1].z-v[1].y*v[0].z;
	        v[2].y = +v[0].z*v[1].x-v[1].z*v[0].x;
	        v[2].z = +v[0].x*v[1].y-v[1].x*v[0].y;
	        //v[2] = normalize(v[2]);

	        for (int i = 0;i<3;i++){
		        m23D[0,i]=v[i].x;
		        m23D[1,i]=v[i].y;
		        m23D[2,i]=v[i].z;
	        }
	        MATINV(3,3,ref m23D,ref vec,out det); 
	        for (int i = 0;i<3;i++){
		        result.simlar[0,i] = (float)(m23D[0,i]*gridDimX);
		        result.simlar[1,i] = (float)(m23D[1,i]*gridDimY);
		        result.simlar[2,i] = (float)(m23D[2,i]*gridDimX*gridDimY);
	        }
	        transformType = ETransformType.TRANSFORM_3D;
	        return result;
        }

		//supporting methods
		public STrackedObject crossPrd(STrackedObject o0,STrackedObject o1,STrackedObject o2,float gridDimX,float gridDimY)
        {
            STrackedObject[] v = new STrackedObject[3];
	        v[0].x = o1.x-o0.x;
	        v[0].y = o1.y-o0.y;
	        v[0].z = o1.z-o0.z;
	        v[0] = normalize(v[0]);

	        v[1].x = o2.x-o0.x;
	        v[1].y = o2.y-o0.y;
	        v[1].z = o2.z-o0.z;
	        v[1] = normalize(v[1]);

	        v[2].x = +v[0].y*v[1].z-v[1].y*v[0].z+o0.x;
	        v[2].y = +v[0].z*v[1].x-v[1].z*v[0].x+o0.y;
	        v[2].z = +v[0].x*v[1].y-v[1].x*v[0].y+o0.z;
	        return v[2];
        }
		
		public float distance (STrackedObject o1, STrackedObject o2)
        {
            return (float)Math.Sqrt((o1.x-o2.x)*(o1.x-o2.x)+(o1.y-o2.y)*(o1.y-o2.y)+(o1.z-o2.z)*(o1.z-o2.z));
        }
		public STrackedObject transformInv(STrackedObject[] o)
        {
            STrackedObject[] trk = new STrackedObject[4];
	        trk[3].x = trk[3].y = trk[3].z = 0;
	        for (int i=0;i<3;i++) trk[i] = o[i];
	        for (int i=0;i<3;i++) trk[i].d = distance(trk[(i+1)%3],trk[(i+2)%3]);
            QSort<STrackedObject>(trk, sortByDistance);
	        //qsort(trk,3,sizeof(STrackedObject),sortByDistance);
	        float an = (float)Math.Atan2(trk[2].x-trk[0].x,trk[2].y-trk[0].y);
            Console.WriteLine("Dock position: {0:F} {0:F} {0:F} {0:F}", trk[0].x, trk[0].y, trk[0].z, 180 * an / Math.PI);
	        //fprintf(stdout,"Dock position: %.3f %.3f %.3f %.3f\n",trk[0].x,trk[0].y,trk[0].z,180*an/M_PI);
	        return o[0];
        }

        //private functions
        private STrackedObject normalize(STrackedObject o)
        {
            float scale = (float)Math.Sqrt(o.x * o.x + o.y * o.y + o.z * o.z);
            STrackedObject r = new STrackedObject();
            r.x = o.x / scale;
            r.y = o.y / scale;
            r.z = o.z / scale;
            return r;
        }
		private float establishError(STrackedObject o)
        {
            STrackedObject result;
            float scale = 0.625f;
            result.x = (float)((o.x / scale - Math.Round(o.x / scale)) * scale);
            result.y = (float)((o.y / scale - Math.Round(o.y / scale)) * scale);
            result.z = (float)((o.z / scale - Math.Round(o.z / scale)) * scale);
            return (float)Math.Sqrt(result.x * result.x + result.y * result.y + result.z * result.z);
        }
		private STrackedObject transform2D(STrackedObject o)
        {
            STrackedObject r = new STrackedObject();
	        r.x = hom[0]*o.x+hom[1]*o.y+hom[2]; 
	        r.y = hom[3]*o.x+hom[4]*o.y+hom[5];
	        r.z = hom[6]*o.x+hom[7]*o.y+hom[8];
	        r.x = r.x/r.z;
	        r.y = r.y/r.z;
	        r.z = 0;
	        r.error = establishError(r);
	        return r;
        }
		private STrackedObject transform3D(STrackedObject o, int num = 4)
        {
            STrackedObject[] result = new STrackedObject[4];
	        STrackedObject final = new STrackedObject();
	        STrackedObject a = new STrackedObject();
	        final.x = final.y = final.z = 0;
	        float str = 0;
	        float strAll = 0;
	        for (int k = 0;k<num;k++){
		        a.x = o.x-D3transform[k].orig.x;
		        a.y = o.y-D3transform[k].orig.y;
		        a.z = o.z-D3transform[k].orig.z;
		        result[k].x = D3transform[k].simlar[0,0]*a.x+D3transform[k].simlar[0,1]*a.y+D3transform[k].simlar[0,2]*a.z;
		        result[k].y = D3transform[k].simlar[1,0]*a.x+D3transform[k].simlar[1,1]*a.y+D3transform[k].simlar[1,2]*a.z;
		        result[k].z = D3transform[k].simlar[2,0]*a.x+D3transform[k].simlar[2,1]*a.y+D3transform[k].simlar[2,2]*a.z;
		        result[k].x = (k%2)*gDimX+(1-(k%2)*2)*result[k].x;
		        result[k].y = (k/2)*gDimY+(1-(k/2)*2)*result[k].y;
		        if (k ==0 || k == 3) result[k].z = -result[k].z;
		        //result.y = +result.y+(D3transform[k].orig.y-D3transform[0].orig.y);
		        //result.z = -result.z+(D3transform[k].orig.z-D3transform[0].orig.z);
		        str=(float)(1.0/(a.x*a.x + a.y*a.y + a.z*a.z+0.01));

		        final.x += str*result[k].x;
		        final.y += str*result[k].y;
		        final.z += str*result[k].z;
		        strAll +=str;
		        //printf("UUU: %f %f %f %f %f\n",result[k].x,result[k].y,result[k].z,str,establishError(result[k]));
	        }
	        final.x=final.x/strAll;
	        final.y=final.y/strAll;	
	        final.z=final.z/strAll;	

	        float x,y,z;
	        final.esterror = 0;
	        for (int k = 0;k<num;k++){
		        x = final.x-result[k].x;
		        y = final.y-result[k].y;
		        z = final.z-result[k].z;
		        final.esterror+=(float)Math.Sqrt(x*x+y*y+z*z);
	        }
	        float xerr0 = -o.z/o.x;
	        float yerr0 = -o.y/o.x;
	        transformXYerr(ref xerr0,ref yerr0);
	        final.esterror= (float)Math.Sqrt(xerr0*xerr0+yerr0*yerr0)*30+fcerr[0]/fc[0]*30;

	        //final.esterror = final.esterror/num;
	        final.error = establishError(final);
	        return final;
        }
		private STrackedObject transform4D(STrackedObject o)
        {
            STrackedObject r = new STrackedObject();
	        r.x = trf4D[0]*o.x+trf4D[1]*o.y+trf4D[2]*o.z+trf4D[3];
	        r.y = trf4D[4]*o.x+trf4D[5]*o.y+trf4D[6]*o.z+trf4D[7]; 
	        r.z = trf4D[8]*o.x+trf4D[9]*o.y+trf4D[10]*o.z+trf4D[11];
	        float s = trf4D[12]*o.x+trf4D[13]*o.y+trf4D[14]*o.z+trf4D[15];
	        r.x = r.x/s;
	        r.y = r.y/s;
	        r.z = r.z/s;
	        r.error = establishError(r);
	        return r;
        }
        #endregion

        #region QSort FUNCTIONS
        public delegate int QSortCompareFunction<T>(T a, T b);
        public static void QSort<T>(T[] array, QSortCompareFunction<T> compareFunc)
        {
            for (int i = 1; i < array.Length; i++)
            {
                T t = array[i];
                int j = i;
                while ((j > 0) && compareFunc(array[j - 1], t) > 0)
                {
                    array[j] = array[j - 1];
                    --j;
                }
                array[j] = t;
            }
        }
        public static int sortByDistance(STrackedObject m1, STrackedObject m2)
        {
            if (m1.d > m2.d) return -1;
            if (m1.d < m2.d) return 1;
            return 0;
        }
        #endregion

        #region LINEAR MATRIX SYSTEM
        private void MATINV(int N, int M, ref double[,] AA, ref double[,] BB, out double DET)
        {
            DET = 1.0;
            double[] PC = new double[NMAX];
            double[] PL = new double[NMAX];
            double[] CS = new double[NMAX];
            double PV,PAV,TT,temp;
            int I,IK,J,JK,K;

            //Initializations:
            for (I=0; I<N; I++)  {                                                                
                PC[I]= 0.0;                                                                
                PL[I]= 0.0;                                                                
                CS[I]= 0.0;                                                                
            }
            //Main K Loop:
            for (K = 0; K < N; K++)
            {
                // Searching greatest pivot :                                               
                PV=AA[K,K];
                IK=K;                                                           
                JK=K;                                                                    
                PAV= Math.Abs(PV);
                for (I=K; I<N; I++)                                                                
                    for (J=K; J<N; J++)  {     
                        temp = Math.Abs(AA[I,J]); 
                        if (temp > PAV) {                                      
                            PV=AA[I,J];                                                        
                            PAV= Math.Abs(PV);                                                      
                            IK=I;                                                              
                            JK=J;                                                              
                            }
                        }
                // Search terminated, the pivot is in location I=IK, J=JK.                     
                // Memorizing pivot location:
	            PC[K]=JK;                                                                
                PL[K]=IK;                                                                                                                               
                // DETERMINANT DET is actualised                                              
                // If DET=0, ERROR MESSAGE and STOP                                           
                // Machine dependant EPSMACH equals here 1e-20
                                                                               
                if (IK!=K) DET=-DET;                                                   
                if (JK!=K) DET=-DET;                                                   
                DET=DET*PV;  
                temp= Math.Abs(DET);                                                            
                if (temp < MACH_EPS) return;
                // POSITIONNING PIVOT IN K,K
                if(IK!=K)                                                         
                    for (I=0; I<N; I++) {                                                              
                        // EXCHANGE LINES IK and K of matrix AA:			
                        TT=AA[IK,I];                                                         
                        AA[IK,I]=AA[K,I];
                        AA[K,I]=TT;                                         
                    }                                               
                                                                           
                if(M!=0)                                                       
                    for (I=0; I<M; I++) {                                                               
                        TT=BB[IK,I];                                                           
                        BB[IK,I]=BB[K,I];                                                      
                        BB[K,I]=TT;                                                            
                    }                                                                   
                                                                           
                // PIVOT is at correct line		  
                if(JK!=K)                                                         
                    for (I=0; I<N; I++) {                                                              
                    // EXCHANGE COLUMNS JK and K of matrix AA:
                        TT=AA[I,JK];                                                         
                        AA[I,JK]=AA[I,K];                                                    
                        AA[I,K]=TT;                                                          
                    }                                                                 
                                                                           
                // The PIVOT is at correct column.                                              
                // and is located in K,K.                                                   
                                                                               
                // Column K of matrix AA is stored in CS vector                             
                // then column K is set to zero. 		  
                for (I=0; I<N; I++) {                                                                
                    CS[I]=AA[I,K];                                                         
                    AA[I,K]= 0.0;                                                          
                    }                                                                   
                                                                               
                CS[K]= 0.0;                                                                
                AA[K,K]= 1.0;                                                              
                // Line K of matrix AA is modified:		
                temp= Math.Abs(PV);                                          
                if(temp < MACH_EPS) return;
                for (I=0; I<N; I++)                                                                
                    AA[K,I]=AA[K,I]/PV;                                                    
                                                                           
                if (M!=0)                                                         
                for (I=0; I<M; I++)                                                             
                    BB[K,I]=BB[K,I]/PV;                                                  
                                                                           
                // Other lines of matrix AA are modified:		  
                for (J=0; J<N; J++) {                                                                
                    if (J==K) J++;                                                  
                    for (I=0; I<N; I++)                                                              
                        // Line J of matrix AA is modified:
                        AA[J,I]=AA[J,I]-CS[J]*AA[K,I];
                    if (M!=0)                                                       
                        for (I=0; I<M; I++)                                                          
                            BB[J,I]=BB[J,I]-CS[J]*BB[K,I];                                     
                }                                                                   
            // Line K is ready
            } //End of K Loop
            // MATRIX AA INVERSION IS DONE - REARRANGEMENT OF MATRIX AA                                                                   
            // EXCHANGE LINES                                                                        
            for (I=N-1; I>=0; I--) {                                                               
                IK=(int) PC[I];                                                                
                if (IK==I) goto fin1;                                                   
                // EXCHANGE LINES I and PC(I) of matrix AA:
                for (J=0; J<N; J++) {                                                                
                    TT=AA[I,J];                                                            
                    AA[I,J]=AA[IK,J];                                                      
                    AA[IK,J]=TT;                                                           
                    }
                if (M!=0)                                                         
                for (J=0; J<M; J++) {                                                             
                    TT=BB[I,J];                                                          
                    BB[I,J]=BB[IK,J];                                                    
                    BB[IK,J]=TT;                                                         
                    }                                                                 
                // NO MORE EXCHANGE is NECESSARY                                                      
                // Go to next line                                                 
                fin1: ;     			  
            } // for i                                                                     
                                                                               
            // EXCHANGE COLUMNS:  	  
            for (J=N-1; J>=0; J--) {                                                                         
                JK=(int) PL[J];                                                                
                if (JK==J) goto fin2;                                                   
                // EXCHANGE COLUMNS J ET PL(J) of matrix AA:
                for (I=0; I<N; I++) {                                                                
                    TT=AA[I,J];                                                            
                    AA[I,J]=AA[I,JK];                                                      
                    AA[I,JK]=TT;                                                           
                 } 
                fin2: ;                                                                 
            // NO MORE EXCHANGE is NECESSARY                                                      
            // Go to next column.
            }                                                                     
        // REARRANGEMENT TERMINATED.                                                        
        return;
        } //MATINV()
        #endregion

        #region MULTIPLICATION OF TWO MATRICES
        private void MATMUL(double[,] A, double[,] B, ref double[,] C, int N)
        {
            double SUM;  
            int I,J,K;                                       
            for (I=0; I<N; I++)                                                                  
                for (J=0; J<N; J++) {                                                                
                    SUM= 0.0;                                                                
                    for (K=0; K<N; K++)                                                              
                        SUM=SUM+A[I,K]*B[K,J];                                               
                        C[I,J]=SUM;                                                            
                }                                                                   
            return;                                                                    
        }
        private void MATMUL1(double[,] A, double[,] B, ref double[,] C, int N, int M)
        {
            double SUM;
            int I,J,K;
            for (I=0; I<N; I++)                                                                  
                for (J=0; J<M; J++) {                                                                
                    SUM= 0.0;                                                                
                    for (K=0; K<N; K++)                                                              
                        SUM=SUM+A[I,K]*B[K,J];                                               
                    C[I,J]=SUM;                                                            
                }                                                                   
            return;
        }
        #endregion
        
        #region EIGEN DECOMPOSITION
        public void eigen_decomposition(double[,] A, ref double[,] V, ref double[] d)
        {
            double[] e = new double[MATSIZE];
            for (int i = 0; i < MATSIZE; i++) {
                for (int j = 0; j < MATSIZE; j++) {
                    V[i,j] = A[i,j];
                }
            }
            tred2(ref V, ref d, ref e);
            tql2(ref V, ref d, ref e);
        }
        static double hypot2(double x, double y)
        {
            return Math.Sqrt(x * x + y * y);
        }
        //symmetric householder reduction to tridiagonal form
        static void tred2(ref double[,] V, ref double[] d, ref double[] e)
        {
            //  This is derived from the Algol procedures tred2 by
            //  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
            //  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
            //  Fortran subroutine in EISPACK.

            for (int j = 0; j < MATSIZE; j++)
            {
                d[j] = V[MATSIZE - 1, j];
            }

            // Householder reduction to tridiagonal form.

            for (int i = MATSIZE - 1; i > 0; i--)
            {

                // Scale to avoid under/overflow.

                double scale = 0.0;
                double h = 0.0;
                for (int k = 0; k < i; k++)
                {
                    scale = scale + Math.Abs(d[k]);
                }
                if (scale == 0.0)
                {
                    e[i] = d[i - 1];
                    for (int j = 0; j < i; j++)
                    {
                        d[j] = V[i - 1, j];
                        V[i, j] = 0.0;
                        V[j, i] = 0.0;
                    }
                }
                else
                {

                    // Generate Householder vector.

                    for (int k = 0; k < i; k++)
                    {
                        d[k] /= scale;
                        h += d[k] * d[k];
                    }
                    double f = d[i - 1];
                    double g = Math.Sqrt(h);
                    if (f > 0)
                    {
                        g = -g;
                    }
                    e[i] = scale * g;
                    h = h - f * g;
                    d[i - 1] = f - g;
                    for (int j = 0; j < i; j++)
                    {
                        e[j] = 0.0;
                    }

                    // Apply similarity transformation to remaining columns.

                    for (int j = 0; j < i; j++)
                    {
                        f = d[j];
                        V[j, i] = f;
                        g = e[j] + V[j, j] * f;
                        for (int k = j + 1; k <= i - 1; k++)
                        {
                            g += V[k, j] * d[k];
                            e[k] += V[k, j] * f;
                        }
                        e[j] = g;
                    }
                    f = 0.0;
                    for (int j = 0; j < i; j++)
                    {
                        e[j] /= h;
                        f += e[j] * d[j];
                    }
                    double hh = f / (h + h);
                    for (int j = 0; j < i; j++)
                    {
                        e[j] -= hh * d[j];
                    }
                    for (int j = 0; j < i; j++)
                    {
                        f = d[j];
                        g = e[j];
                        for (int k = j; k <= i - 1; k++)
                        {
                            V[k, j] -= (f * e[k] + g * d[k]);
                        }
                        d[j] = V[i - 1, j];
                        V[i, j] = 0.0;
                    }
                }
                d[i] = h;
            }

            // Accumulate transformations.

            for (int i = 0; i < MATSIZE - 1; i++)
            {
                V[MATSIZE - 1, i] = V[i, i];
                V[i, i] = 1.0;
                double h = d[i + 1];
                if (h != 0.0)
                {
                    for (int k = 0; k <= i; k++)
                    {
                        d[k] = V[k, i + 1] / h;
                    }
                    for (int j = 0; j <= i; j++)
                    {
                        double g = 0.0;
                        for (int k = 0; k <= i; k++)
                        {
                            g += V[k, i + 1] * V[k, j];
                        }
                        for (int k = 0; k <= i; k++)
                        {
                            V[k, j] -= g * d[k];
                        }
                    }
                }
                for (int k = 0; k <= i; k++)
                {
                    V[k, i + 1] = 0.0;
                }
            }
            for (int j = 0; j < MATSIZE; j++)
            {
                d[j] = V[MATSIZE - 1, j];
                V[MATSIZE - 1, j] = 0.0;
            }
            V[MATSIZE - 1, MATSIZE - 1] = 1.0;
            e[0] = 0.0;
        }
        // Symmetric tridiagonal QL algorithm.

        static void tql2(ref double[,] V, ref double[] d, ref double[] e)
        {
            //  This is derived from the Algol procedures tql2, by
            //  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
            //  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
            //  Fortran subroutine in EISPACK.

            for (int i = 1; i < MATSIZE; i++)
            {
                e[i - 1] = e[i];
            }
            e[MATSIZE - 1] = 0.0;

            double f = 0.0;
            double tst1 = 0.0;
            double eps = Math.Pow(2.0, -52.0);
            for (int l = 0; l < MATSIZE; l++)
            {

                // Find small subdiagonal element

                tst1 = Math.Max(tst1, Math.Abs(d[l]) + Math.Abs(e[l]));
                int m = l;
                while (m < MATSIZE)
                {
                    if (Math.Abs(e[m]) <= eps * tst1)
                    {
                        break;
                    }
                    m++;
                }
                while (m >= MATSIZE) m--;
                // If m == l, d[l] is an eigenvalue,
                // otherwise, iterate.

                if (m > l)
                {
                    int iter = 0;
                    do
                    {
                        iter = iter + 1;  // (Could check iteration count here.)

                        // Compute implicit shift

                        double g = d[l];
                        double p = (d[l + 1] - g) / (2.0 * e[l]);
                        double r = hypot2(p, 1.0);
                        if (p < 0)
                        {
                            r = -r;
                        }
                        d[l] = e[l] / (p + r);
                        d[l + 1] = e[l] * (p + r);
                        double dl1 = d[l + 1];
                        double h = g - d[l];
                        for (int i = l + 2; i < MATSIZE; i++)
                        {
                            d[i] -= h;
                        }
                        f = f + h;

                        // Implicit QL transformation.

                        p = d[m];
                        double c = 1.0;
                        double c2 = c;
                        double c3 = c;
                        double el1 = e[l + 1];
                        double s = 0.0;
                        double s2 = 0.0;
                        for (int i = m - 1; i >= l; i--)
                        {
                            c3 = c2;
                            c2 = c;
                            s2 = s;
                            g = c * e[i];
                            h = c * p;
                            r = hypot2(p, e[i]);
                            e[i + 1] = s * r;
                            s = e[i] / r;
                            c = p / r;
                            p = c * d[i] - s * g;
                            d[i + 1] = h + s * (c * g + s * d[i]);

                            // Accumulate transformation.

                            for (int k = 0; k < MATSIZE; k++)
                            {
                                h = V[k, i + 1];
                                V[k, i + 1] = s * V[k, i] + c * h;
                                V[k, i] = c * V[k, i] - s * h;
                            }
                        }
                        p = -s * s2 * c3 * el1 * e[l] / dl1;
                        e[l] = s * p;
                        d[l] = c * p;

                        // Check for convergence.

                    } while (Math.Abs(e[l]) > eps * tst1);
                }
                d[l] = d[l] + f;
                e[l] = 0.0;
            }

            // Sort eigenvalues and corresponding vectors.

            for (int i = 0; i < MATSIZE - 1; i++)
            {
                int k = i;
                double p = d[i];
                for (int j = i + 1; j < MATSIZE; j++)
                {
                    if (d[j] < p)
                    {
                        k = j;
                        p = d[j];
                    }
                }
                if (k != i)
                {
                    d[k] = d[i];
                    d[i] = p;
                    for (int j = 0; j < MATSIZE; j++)
                    {
                        p = V[j, i];
                        V[j, i] = V[j, k];
                        V[j, k] = p;
                    }
                }
            }
        }
        #endregion
    }

    //QSort Class in C#
    class QSort<T> where T : IComparable
    {
        public IList<T> A;
        public QSort(IList<T> A)
        {
            this.A = A;
        }
        public int Partition(int L, int U)
        {
            int s = U;
            int p = L;
            while (s != p)
            {
                if (A[p].CompareTo(A[s]) <= 0)
                {
                    p++;
                }
                else
                {
                    Swap(p, s);
                    Swap(p, s - 1);
                    s--;
                }
            }
            return p;
        }
        private void Swap(int p, int s)
        {
            T tmp = A[p];
            A[p] = A[s];
            A[s] = tmp;
        }
        public void Sort(int L, int U)
        {
            if (L >= U)
                return;
            int p = Partition(L, U);
            Sort(L, p - 1);
            Sort(p + 1, U);
        }
        public void Sort()
        {
            Sort(0, A.Count - 1);
        }
    }
}
