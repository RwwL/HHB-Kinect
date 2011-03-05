#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	//kinect.init(true);  //shows infrared image
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();

	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThresh.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	nearThreshold = 230;
	farThreshold  = 70;
	bThreshWithOpenCV = false;
	
	ofSetFrameRate(60);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	pointCloudRotationY = 180;
	
	drawPC = false;
	
	testInt = 0;
	myMinArea = 15000; // thinking about 150x100px might be good to pick up a humanoid blob?
	myMaxArea = 90000; // 300x300
	
}

//--------------------------------------------------------------
void testApp::update() {
	ofBackground(100, 100, 100);
	
	kinect.update();
	if(kinect.isFrameNew())	// there is a new frame and we are connected
	{

		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
			
		//we do two thresholds - one for the far plane and one for the near plane
		//we then do a cvAnd to get the pixels which are a union of the two thresholds.	
		if( bThreshWithOpenCV ){
			grayThreshFar = grayImage;
			grayThresh = grayImage;
			grayThresh.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThresh.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		}else{
		
			//or we do it ourselves - show people how they can work with the pixels
		
			unsigned char * pix = grayImage.getPixels();
			int numPixels = grayImage.getWidth() * grayImage.getHeight();

			for(int i = 0; i < numPixels; i++){
				if( pix[i] < nearThreshold && pix[i] > farThreshold ){
					pix[i] = 255;
				}else{
					pix[i] = 0;
				}
			}
		}

		//update the cv image
		grayImage.flagImageChanged();
	
		// find contours which are between the size of my defined min and max areas.
    	// also, set find holes to false so we wont' get interior contours ... not sure what bUseApproximation does but it's safe to omit
		// kinect.width = 640, kinect.height = 480, by the wa 
    	contourFinder.findContours(grayImage, myMinArea, myMaxArea, 20, false);
		//contourFinder.findContours(<#ofxCvGrayscaleImage input#>, int minArea, <#int maxArea#>, <#int nConsidered#>, <#bool bFindHoles#>, <#bool bUseApproximation#>)
	}
	
	//testInt ++;
	
	//if (testInt == 100)
//	{
//		// this works; sends A to app that has keyboard focus:
//		CGPostKeyboardEvent( (CGCharCode)'a', (CGKeyCode)0, true);
//	}
//	if (testInt == 200)
//	{
//		// this works; sends L to app that has keyboard focus:
//		CGPostKeyboardEvent( (CGCharCode)'l', (CGKeyCode)37, true );
//		testInt = 0;
//	}
	
}

//--------------------------------------------------------------
void testApp::draw() {
	ofSetColor(255, 255, 255);
	if(drawPC){
		ofPushMatrix();
		ofTranslate(420, 320);
		// we need a proper camera class
		drawPointCloud();
		ofPopMatrix();
	}else{
		stringstream drawDepthLabel;
		drawDepthLabel << "kinect.drawDepth";
		ofDrawBitmapString(drawDepthLabel.str(), 10, 10);
		kinect.drawDepth(10, 15, 400, 300);
		
		stringstream drawLabel;
		drawLabel << "kinect.draw";
		ofDrawBitmapString(drawLabel.str(), 420, 10	);
		kinect.draw(420, 15, 400, 300);

		stringstream grayLabel;
		grayLabel << "grayImage.draw + contourFinder.draw";
		ofDrawBitmapString(grayLabel.str(), 10, 330);
		grayImage.draw(10, 335, 400, 300);
		contourFinder.draw(10, 335, 400, 300);
	}
	

	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "using opencv threshold = " << bThreshWithOpenCV << " (press spacebar) " << endl
				<< endl
				<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
				<< endl
				<< "set near threshold " << nearThreshold << " (press: + -)" << "    ||    " << "set far threshold " << farThreshold << " (press: < >) " << endl
				<< endl
				<< "num blobs found: " << contourFinder.nBlobs << endl
				<< endl
				<< "press c to close the connection and k to open it again, connection is: " << kinect.isConnected() << endl
				<< endl
				<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
				<< endl
				<< "myMinArea (o/p): " << myMinArea << ", myMaxArea ([/]): " << myMaxArea;
				
				// reporting I don't really need for now
				// ", fps: " << ofGetFrameRate() 
				//	<< "accel is: "
				//	<< ofToString(kinect.getMksAccel().x, 2) << " / "
				//	<< ofToString(kinect.getMksAccel().y, 2) << " / " 
				//	<< ofToString(kinect.getMksAccel().z, 2) << endl
	
	
	ofDrawBitmapString(reportStream.str(),20,666);
}

void testApp::drawPointCloud() {
	ofScale(400, 400, 400);
	int w = 640;
	int h = 480;
	ofRotateY(pointCloudRotationY);
	float* distancePixels = kinect.getDistancePixels();
	glBegin(GL_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			ofPoint cur = kinect.getWorldCoordinateFor(x, y);
			ofColor color = kinect.getCalibratedColorAt(x,y);
			glColor3ub((unsigned char)color.r,(unsigned char)color.g,(unsigned char)color.b);
			glVertex3f(cur.x, cur.y, cur.z);
		}
	}
	glEnd();
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case '[':
			myMaxArea--;
			break;
		case ']':
			myMaxArea++;
			break;
		case 'o':
			myMinArea--;
			break;
		case 'p':
			myMinArea++;
			break;
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
		case'q':
			drawPC = !drawPC;
			break;
	
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
		case '<':		
		case ',':		
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
		case '-':		
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
		case 'k':
			kinect.setCameraTiltAngle(angle);	// go back to prev tilt
			kinect.open();
			break;
		case 'c':
			kinect.setCameraTiltAngle(0);		// zero the tilt
			kinect.close();
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y) {
	pointCloudRotationY = x;
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}

