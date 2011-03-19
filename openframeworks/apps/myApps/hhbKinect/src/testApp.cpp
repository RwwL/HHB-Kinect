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

	nearThreshold = 255;
	farThreshold  = 140;
	bThreshWithOpenCV = false;
	
	ofSetFrameRate(20);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	pointCloudRotationY = 180;
		
	drawPC = false;
	
	testInt = 0;
	myMinArea = 18000; // guessing at these... no rhyme or reason here, thank you very much
	myMaxArea = 95000;
	
	observingLeft = true;
	observingRight = true;
	blobOnLeft = false;
	blobOnRight = false;
	centerLine = kinect.width/2;
	sideBuffer = 100;
	
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
			// [RL] USING THIS ONE... not that it probably matters too much
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
		// not sure what nConsidered does yet; leaving it alone...
    	// also, set find holes to false so we wont' get interior contours ... not sure what bUseApproximation does but it's safe to omit
		// kinect.width = 640, kinect.height = 480, by the way
    	contourFinder.findContours(grayImage, myMinArea, myMaxArea, 20, false);
		//contourFinder.findContours( ofxCvGrayscaleImage input, int minArea, int maxArea, int nConsidered, bool bFindHoles, bool bUseApproximation )

	}
	
	blobOnLeft = false;
	blobOnRight = false;
	
	for (int i = 0; i < contourFinder.blobs.size(); i++) {
		
		int blobCtrX = contourFinder.blobs[i].centroid.x;
		
		if (blobCtrX < centerLine && blobCtrX > sideBuffer)
		{
			blobOnLeft = true;
		}
		
		if (blobCtrX > centerLine && blobCtrX < kinect.width - sideBuffer)
		{
			blobOnRight = true;
		}
		
	}
	
	
	if (blobOnLeft && observingLeft)
	{
		// fire the A key!
		CGPostKeyboardEvent( (CGCharCode)'a', (CGKeyCode)0, true);
		CGPostKeyboardEvent( (CGCharCode)'a', (CGKeyCode)0, false);
		// prevent firing on next update so the user has to get out of frame
		observingLeft = false;
	}
	if (blobOnRight && observingRight)
	{
		// fire the L key! 
		CGPostKeyboardEvent( (CGCharCode)'l', (CGKeyCode)37, true );
		CGPostKeyboardEvent( (CGCharCode)'l', (CGKeyCode)37, false );
		// prevent firing on next update so the user has to get out of frame
		observingRight = false;
	}
	
	// no blobs seen on a given side? start observing again
	if (!blobOnLeft) {
		observingLeft = true;
	}
	
	if (!blobOnRight) {
		observingRight = true;
	}
	
	
	// this works; sends A to app that has keyboard focus:
	// CGPostKeyboardEvent( (CGCharCode)'a', (CGKeyCode)0, true);
	// this works; sends L to app that has keyboard focus:
	// CGPostKeyboardEvent( (CGCharCode)'l', (CGKeyCode)37, true );
	
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
		//stringstream drawDepthLabel;
		//drawDepthLabel << "kinect.drawDepth";
		//ofDrawBitmapString(drawDepthLabel.str(), 10, 10);
		//kinect.drawDepth(10, 15, 400, 300);
		
		//stringstream drawLabel;
		//drawLabel << "kinect.draw";
		//ofDrawBitmapString(drawLabel.str(), 420, 10	);
		//kinect.draw(420, 15, 400, 300);

		stringstream grayLabel;
		grayLabel << "grayImage.draw + contourFinder.draw";
		ofDrawBitmapString(grayLabel.str(), 10, 20); // y was 330
		grayImage.draw(10, 25, 400, 300); // y was 335
		contourFinder.draw(10, 25, 400, 300); // y was 335
	}
	

	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream    << "observingLeft: " << observingLeft << endl
                    << endl
                    << "blobOnLeft: " << blobOnLeft << endl
                    << endl
                    << "--------------------" << endl
                    << endl
                    << "observingRight: " << observingRight << endl
                    << endl
                    << "blobOnRight: " << blobOnRight << endl
                    << endl
                    << "--------------------" << endl
                    << endl
                    << "near threshold (+ / -): " << nearThreshold << endl
                    << endl
                    << "far threshold  (< / >): " << farThreshold << endl
                    << endl
                    << "sideBuffer (u / i): " << sideBuffer << endl                
                    << endl
                    << "tilt angle (up arrow / down arrow): " << angle << " degrees" << endl
                    << endl
                    << "minArea (o / p): " << myMinArea << endl 
                    << endl
                    << "maxArea ([ / ]): " << myMaxArea << endl
                    << endl
                    << "centerLine: " << centerLine << endl
                    << endl
                    << "press c to close the connection and k to open it again" << endl
                    << "connection is: " << kinect.isConnected();

                    ofDrawBitmapString(reportStream.str(),420,35);

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
			myMaxArea-=100;
			break;
		case ']':
			myMaxArea+=100;
			break;
		case 'o':
			myMinArea-=100;
			break;
		case 'p':
			myMinArea+=100;
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
		case 'i':
			sideBuffer ++;
			if (sideBuffer > centerLine) sideBuffer = centerLine;
			break;
		case 'u':		
			sideBuffer --;
			if (sideBuffer < 0) sideBuffer = 0;
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

