#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

	ofSetLogLevel(OF_LOG_NOTICE);
	ofSetFrameRate(60);
	drawWidth = 1024;
	drawHeight = 768;




	// SOUND
	sampleRate = 44100;
    channels = 2;
    soundStream.setup(this, 0, channels, sampleRate, 256, 4);

    stageImg.allocate(drawWidth,drawHeight,ofImageType::OF_IMAGE_COLOR_ALPHA);

	//------------------------------------
	// KINECT STUFF
	kinect.setRegistration(true);
	kinect.init(false, false);
	kinect.open();
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	ofLog(OF_LOG_NOTICE,"kinectWidth="+ofToString(grayImage.width));
	ofLog(OF_LOG_NOTICE,"kinectHeight="+ofToString(grayImage.height));
	
	nearThreshold = 190;
	farThreshold = 125;
	bThreshWithOpenCV = false;

	kinectFbo.allocate(drawWidth, drawHeight, GL_RGBA32F);

	//------------------------------------
	// SIMULATION STUFF
	flowWidth = drawWidth / 4;
	flowHeight = drawHeight /4;
	tmpFbo.allocate(drawWidth, drawHeight, GL_RGBA32F);

	// flow, mask, simulation
	opticalFlow.setup(flowWidth, flowHeight);
	velocityMask.setup(flowWidth, flowHeight);
	fluidSimulation.setup(flowWidth, flowHeight, drawWidth, drawHeight);
	fluidSimulation.setDissipation(0.001);
	fluidSimulation.setGravity(ofVec2f(0,5));
	fluidSimulation.setViscosity(0.1);
	particleFlow.setup(flowWidth, flowHeight, drawWidth, drawHeight);

	// LOGO
	images.push_back("logo_sequence.png");
	images.push_back("logo_css.png");
	showLogo = true;
	logoID = 1;	
		logo.load(images[logoID]);
	logo.resize(drawWidth, drawHeight);
	tex = logo.getTexture();
	logoID = 0;

	// VISUALIZATION
	displayScalar.setup(flowWidth, flowHeight);
	velocityField.setup(flowWidth / 4, flowHeight / 4);
	temperatureField.setup(flowWidth / 4, flowHeight / 4);
	pressureField.setup(flowWidth / 4, flowHeight / 4);
	velocityTemperatureField.setup(flowWidth / 4, flowHeight / 4);

	


	// time
	t = ofGetElapsedTimef();
	
	getXMLsources();

	updateSourceMode();

	initsources();

	initflow();

}

//--------------------------------------------------------------
void ofApp::initflow(){

	// fluidSimulation
	fluidSimulation.reset();
	// fluidSimulation.setup(flowWidth, flowHeight, drawWidth, drawHeight);
	// fluidSimulation.setDissipation(0.001);
	// fluidSimulation.setGravity(ofVec2f(0,5));
	// fluidSimulation.setViscosity(0.1);

	if (logoID<images.size())
	{
		showLogo = true;

		logo.clear();	
		logo.load(images[logoID]);
		logo.resize(drawWidth, drawHeight);
		
		ofSetColor(255);
		tex = logo.getTexture();


		// fluidSimulation.addObstacle(logo.getTexture());
		fluidSimulation.addObstacle(tex);

		
	}	
	else
	{
		showLogo = false;
	}
}

//--------------------------------------------------------------
void ofApp::initsources(){

	// forces	
	drawForces = new ftDrawForce[3];
	drawForces[0].setup(drawWidth, drawHeight, FT_DENSITY, true);
	drawForces[0].setName("draw density");
	drawForces[1].setup(drawWidth, drawHeight, FT_VELOCITY, true);
	drawForces[1].setName("draw velocity");
	drawForces[2].setup(drawWidth, drawHeight, FT_TEMPERATURE, true);
	drawForces[2].setName("draw temperature");

	int N;
	
	if (sourceMode == 0)
	{
		N = NS1;
	}
	else
	{
		N = sourceX.size();
	}

	ofFloatColor c;
	if (sourceMode == 1)
	{
		c = c1;
	}
	else if (sourceMode ==2 )

	{
		c = c2;
	}

	for (int i=0; i<N; i++)
	{
		drawForces[0].setRadius(radii[i]);
		if (i<NS1)
		{
			drawForces[0].setForce(colors[i]);	
		}
		else
		{
			drawForces[0].setForce(c);
		}
		
		drawForces[0].applyForce(ofVec2f(sourceX[i],sourceY[i]));
		
		drawForces[1].setRadius(radii[i]/2);
		drawForces[1].setForce(ofVec2f(velocityX[i],velocityY[i]));
		drawForces[1].applyForce(ofVec2f(sourceX[i],sourceY[i]));
	}

	drawForces[2].applyForce(ofVec2f(-0.1,0.5));
}



//--------------------------------------------------------------
void ofApp::update(){
	ofSetWindowTitle("FPS="+ofToString(round(ofGetFrameRate()))+
		"; nearThresh="+ ofToString(nearThreshold)+
		"; farThresh=" + ofToString(farThreshold));

	// time
	dt = ofGetElapsedTimef() - t;
	t = ofGetElapsedTimef();

	updateSourceMode();

	// cycle logos every so often
	t_ms = ofGetElapsedTimeMillis();
	int t_s = t_ms/1000;
	if (t_s>0 && t_s % (60*5) == 0)
	{
		if (!(fadeOut||fadeIn ))
		{
			ofLog(OF_LOG_NOTICE,"cycle logo");
			cycleLogo();
		}
		
	}

	float minBright = 0.01;
	if (fadeOut)
	{
		if(mastBright>=minBright)
		{
			mastBright *= 0.99;
		}
		else
		{
			fadeOut = false;
			fadeIn = true;
			doReset = true;
		}			
	}
	if (fadeIn)
	{
		if (mastBright==0)
		{
			mastBright = minBright;
		}

		if(mastBright<1.0)
		{
			mastBright*=1.01;			
		}
		else
		{
			fadeIn = false;
		}
	}

	if (doReset)
	{
		doReset = false;
		initsources();
		initflow();
	}

	// KINECT
	kinect.update();

	if (kinect.isFrameNew())
	{
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());
		grayImage.mirror(false,true);
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		// if(bThreshWithOpenCV) {
		// 	grayThreshNear = grayImage;
		// 	grayThreshFar = grayImage;
		// 	grayThreshNear.threshold(nearThreshold, true);
		// 	grayThreshFar.threshold(farThreshold);
		// 	cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		// }	
		// or we do it ourselves - show people how they can work with the pixels
		ofPixels & pix = grayImage.getPixels();
		int numPixels = pix.size();
		for(int i = 0; i < numPixels; i++) {
			if(pix[i] < nearThreshold && pix[i] > farThreshold) {
				pix[i] = 255;
			} else {
				pix[i] = 0;
			}
		}

		// update the cv images
		grayImage.flagImageChanged();
	}
	
	kinectFbo.begin();
	ofClear(0,0,0,0);
	ofPushMatrix();
	// ofTranslate(192, 288);

	float tmpScale = 1.09;
	float x0 = -35;
	float y0 = -100;
	grayImage.draw(x0,y0,tmpScale*drawWidth,tmpScale*drawHeight);
	ofPopMatrix();
	kinectFbo.end();
	

	// forces
	for (int i=0; i< nForces; i++)
	{
		drawForces[i].update();		
	}
	fluidSimulation.addDensity(drawForces[0].getTexture(), drawForces[0].getStrength());
	fluidSimulation.addVelocity(drawForces[1].getTexture(), drawForces[1].getStrength());
	fluidSimulation.addTemperature(drawForces[2].getTexture(), drawForces[2].getStrength());

	// kinect and optical flow
	if (kinect.isFrameNew())
	{
		opticalFlow.setSource(kinectFbo.getTexture());
		opticalFlow.update();
		velocityMask.setDensity(kinectFbo.getTexture());
		velocityMask.update();
	}

	// update fluid simulation
	fluidSimulation.addVelocity(opticalFlow.getOpticalFlowDecay());
	fluidSimulation.addDensity(velocityMask.getColorMask());
	fluidSimulation.addTemperature(velocityMask.getLuminanceMask());
	fluidSimulation.update();

	updateConstantSources();


}

void ofApp::updateSourceMode()
{
	t_hours = ofGetHours();
	t_mins = ofGetMinutes();
	if (t_hours>22 && (t_hours <= 23 && t_mins<30 ))
	{
		sourceMode = 0;
	}
	else if((t_hours>=23 && t_mins>=30) || t_hours<1)
	{
		sourceMode = 1;		
	}
	else
	{
		sourceMode = 2;
	}

}

//--------------------------------------------------------------
void ofApp::draw(){
	ofClear(0,0);
	ofSetColor(255,255,255,mastBright*255);

	drawComposite();

	// vidGrabber.draw(0,0);

	// if (kinect.isFrameNew())
	{
		if(drawDepth)
		{
			if (debug)
			{
				ofEnableBlendMode(OF_BLENDMODE_ADD);
			}
			else
			{
				ofEnableBlendMode(OF_BLENDMODE_SUBTRACT);
			}
			// ofPushMatrix();
			// ofTranslate(192, 288);

			// grayImage.draw(0,0);
			// ofPopMatrix();

			// ofDrawEllipse(mouseX, mouseY, 100, 100);

			kinectFbo.draw(0,0);
			
			// ofEnableBlendMode(OF_BLENDMODE_ALPHA);

			// kinectFbo.getTexture().draw(0,0);

			ofEnableBlendMode(OF_BLENDMODE_ADD);
		}
	}
	if (showLogo)
	{
		// ofEnableAlphaBlending();
		ofSetColor(255,255,255,255*mastBright);
		logo.draw(0, 0 , drawWidth , drawHeight);
		// ofDisableAlphaBlending();
	}

	if (debug)
	{
		ofSetColor(255);
		ofDrawBitmapString("FPS = "+ofToString(round(ofGetFrameRate())), 10, 20);
		ofDrawBitmapString("nearThresh = " + ofToString(nearThreshold), 10, 40);
		ofDrawBitmapString("farThresh = " + ofToString(farThreshold), 10, 60);
		ofDrawBitmapString("logoID = " + ofToString(logoID), 10, 80);
		ofDrawBitmapString("sourceMode = " + ofToString(sourceMode), 10, 100);

		ofDrawBitmapString("t=" + ofToString(t),drawWidth-80,20);
		ofDrawBitmapString("t_ms="+ ofToString(t_ms),drawWidth-80,40);
		ofDrawBitmapString(ofToString(t_ms/1000 % (60)),drawWidth-80,60);

		ofDrawBitmapString("SYS "+ofToString(t_hours) + ":" + ofToString(t_mins),drawWidth-80,90);


		ofSetColor(255);
		tex.draw(924,668,100,100);
	}
}

//--------------------------------------------------------------
void ofApp::updateConstantSources()
{	

	if (startFlow)
	{
	
		float T = 2*0.97;

		float s = 0.3*pow(sin(1.0*3.14*t/T),16);

		drawForces[0].setStrength(s);

		float v = 1 + 0.5*sin(1.0*3.14*t/T);
		drawForces[1].setStrength(v);
	}
	else
	{
		drawForces[0].setStrength(0);
		drawForces[1].setStrength(1);
	}


}

//--------------------------------------------------------------
void ofApp::drawComposite(int x, int y, int w, int h){
	ofPushStyle();
	
	ofEnableBlendMode(OF_BLENDMODE_ADD);
	fluidSimulation.draw(x,y,w,h);

	ofEnableBlendMode(OF_BLENDMODE_ADD);
	
	
	ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::cycleLogo()
{
	logoID++;
	logoID = logoID%(images.size() + 1);
	fadeOut = true;
}

//--------------------------------------------------------------
void ofApp::getXMLsources()
{
	// XML SOURCES
	if (XML.load("source_config.xml"))
	{
		ofLog(OF_LOG_NOTICE,"Loaded XML sources");
	}
	else
	{
		ofLog(OF_LOG_ERROR,"Failed to load XML sources");
	}



	

	if(XML.exists("SOURCE"))
	{
		ofLog(OF_LOG_NOTICE,"SOURCE");
		if(XML.setTo("SOURCE[0]"))
		{
			do
				{
					double x = XML.getValue<double>("X");
					sourceX.push_back(x);
					double y = XML.getValue<double>("Y");
					sourceY.push_back(y);
					double vX = XML.getValue<double>("vX");
					velocityX.push_back(vX);
					double vY = XML.getValue<double>("vY");
					velocityY.push_back(vY);
					double R = XML.getValue<double>("R");
					radii.push_back(R);
					double cR = XML.getValue<double>("cR");
					double cG = XML.getValue<double>("cG");
					double cB = XML.getValue<double>("cB");
					double cA = XML.getValue<double>("cA");
					colors.push_back(ofFloatColor(cR,cG,cB,cA));
				}
				while(XML.setToSibling());
				XML.setToParent();
		}		
	}

	if(XML.exists("PARAMS"))
	{
		ofLog(OF_LOG_NOTICE,"PARAMS");
		
		XML.setTo("PARAMS[0]");

		NS1 = XML.getValue<int>("NS1");

		
		float cR1 = XML.getValue<float>("cR1");
		float cG1 = XML.getValue<float>("cG1");
		float cB1 = XML.getValue<float>("cB1");
		float cA1 = XML.getValue<float>("cA1");
		c1 = ofFloatColor(cR1,cG1,cB1,cA1);

		float cR2 = XML.getValue<float>("cR2");
		float cG2 = XML.getValue<float>("cG2");
		float cB2 = XML.getValue<float>("cB2");
		float cA2 = XML.getValue<float>("cA2");
		c2 = ofFloatColor(cR2,cG2,cB2,cA2);

		ofLog(OF_LOG_NOTICE,"cR2="+ofToString(cR2));
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

	if(key=='r')
	{
		doReset = true;
		showLogo = !showLogo;
	}
	
	if(key==OF_KEY_UP)
	{
		farThreshold += threshStep;
	}
	if(key==OF_KEY_DOWN)
	{
		farThreshold -= threshStep;
	}
	if(key==OF_KEY_LEFT)
	{
		nearThreshold -= threshStep;
	}
	if(key==OF_KEY_RIGHT)
	{
		nearThreshold += threshStep;
	}
	if(key==OF_KEY_SHIFT)
	{
		threshStep = 10;
	}

	if(key =='d')
	{
		debug = !debug;
	}

	if(key == '1')
	{
		startFlow = !startFlow;
	}

	if(key==' ')
	{
		drawDepth = !drawDepth;
	}

	if(key=='f')
	{
		fadeIn = false;
		fadeOut = true;
	}

	if(key=='2')
	{
		if (!(fadeIn||fadeOut))
		cycleLogo();
		
	}

	if(key=='3')
	{
		sourceMode++;
		sourceMode = sourceMode%3;
		fadeOut = true;
		// change source mode
	}
}

//--------------------------------------------------------------
void ofApp::audioIn(float *input, int bufferSize, int nChannels){

}


//--------------------------------------------------------------
void ofApp::keyReleased(int key){
	if (key==OF_KEY_SHIFT)
	{
		threshStep = 1;
	}	
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

//--------------------------------------------------------------
void ofApp::exit(){

}