#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

	ofSetLogLevel(OF_LOG_NOTICE);
	ofSetFrameRate(60);
	drawWidth = 1024;
	drawHeight = 768;

	// VIDEO RECORDING
	sampleRate = 44100;
    channels = 2;
    // vidGrabber.setDesiredFrameRate(30);
    // vidGrabber.initGrabber(drawWidth, drawHeight);
    fileName = "testMovie";
    fileExt = ".mov"; // ffmpeg uses the extension to determine the container type. run 'ffmpeg -formats' to see supported formats

    // override the default codecs if you like
    // run 'ffmpeg -codecs' to find out what your implementation supports (or -formats on some older versions)
    vidRecorder.setVideoCodec("mpeg4");
    vidRecorder.setVideoBitrate("800k");
    vidRecorder.setAudioCodec("mp3");
    vidRecorder.setAudioBitrate("192k");
    soundStream.setup(this, 0, channels, sampleRate, 256, 4);


    ofAddListener(vidRecorder.outputFileCompleteEvent, this, &ofApp::recordingComplete);

    bRecording = false;	

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

	// forces
	drawForces = new ftDrawForce[3];
	drawForces[0].setup(drawWidth, drawHeight, FT_DENSITY, true);
	drawForces[0].setName("draw density");
	drawForces[1].setup(drawWidth, drawHeight, FT_VELOCITY, true);
	drawForces[1].setName("draw velocity");
	drawForces[2].setup(drawWidth, drawHeight, FT_TEMPERATURE, true);
	drawForces[2].setName("draw temperature");

	// flow, mask, simulation
	opticalFlow.setup(flowWidth, flowHeight);
	velocityMask.setup(flowWidth, flowHeight);
	fluidSimulation.setup(flowWidth, flowHeight, drawWidth, drawHeight);
	fluidSimulation.setDissipation(0.001);
	fluidSimulation.setGravity(ofVec2f(0,5));
	particleFlow.setup(flowWidth, flowHeight, drawWidth, drawHeight);

	// LOGO
	logo.load("logo_OASC.png");
	logo.resize(drawWidth, drawHeight);
	fluidSimulation.addObstacle(logo.getTexture());
	showLogo = true;

	// VISUALIZATION
	displayScalar.setup(flowWidth, flowHeight);
	velocityField.setup(flowWidth / 4, flowHeight / 4);
	temperatureField.setup(flowWidth / 4, flowHeight / 4);
	pressureField.setup(flowWidth / 4, flowHeight / 4);
	velocityTemperatureField.setup(flowWidth / 4, flowHeight / 4);

	// KINECT

	// time
	t = ofGetElapsedTimef();

	sourceRadius = 0.1;
	for (int i=0; i<nForces; i++)
	{
		
	}

	// sourceX.push_back(0.1);
	// sourceY.push_back(0.5);
	// velocityX.push_back(0.01);
	// velocityY.push_back(-0.005);

	// sourceX.push_back(0.9);
	// sourceY.push_back(0.5);
	// velocityX.push_back(-0.01);
	// velocityY.push_back(-0.005);

	// LOWERS
	sourceX.push_back(0.15);
	sourceY.push_back(1);
	velocityX.push_back(0.0001);
	velocityY.push_back(-0.002);
	radii.push_back(0.12);
	colors.push_back(ofFloatColor(0.9,0.82,0.17,0.8));

	sourceX.push_back(0.5);
	sourceY.push_back(1.05);
	velocityX.push_back(0);
	velocityY.push_back(-0.038);
	radii.push_back(0.16);
	colors.push_back(ofFloatColor(0.85,0.01,0.01,0.8));

	sourceX.push_back(0.85);
	sourceY.push_back(1);
	velocityX.push_back(-0.0001);
	velocityY.push_back(-0.002);
	radii.push_back(0.12);
	// colors.push_back(ofFloatColor(0.2,0.0,0.8,0.8));
	colors.push_back(ofFloatColor(0.9,0.82,0.17,0.8));

	// UPPERS
	sourceX.push_back(0.4);
	sourceY.push_back(-0.01);
	velocityX.push_back(-0.001);
	velocityY.push_back(0.0015);
	radii.push_back(0.15);
	colors.push_back(ofFloatColor(0.55,0.9,1.0,0.2));
	// colors.push_back(ofFloatColor(0.8,0.8,0.8,0.8));

	sourceX.push_back(0.6);
	sourceY.push_back(-0.0);
	velocityX.push_back(0.001);
	velocityY.push_back(0.0015);
	radii.push_back(0.15);
	// colors.push_back(ofFloatColor(0.59,0.0196,0.1,0.8));
	colors.push_back(ofFloatColor(0.55,0.9,1.0,0.2));
	
	
	
	
	
	


	// for (int i=0; i<sourceX.size(); i++)
	for (int i=0; i<3; i++)
	{
		drawForces[0].setRadius(radii[i]);
		drawForces[0].setForce(colors[i]);
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

	// VIDEO
 	// vidGrabber.update();
    if(bRecording){
        // bool success = vidRecorder.addFrame(vidGrabber.getPixels());
        
        stageImg.getTexture().loadScreenData(0, 0, drawWidth, drawHeight);
		stageImg.getTexture().readToPixels(stagePixels);		
		// flip vertically; OpenGL uses opposite convention than OFX
		stagePixels.mirror(true, false);

		// ofSaveImage(stagePixels,"blah"+ofToString(nframes,4,'0')+".png");
		// nframes++;
		bool success = vidRecorder.addFrame(stagePixels);

        if (!success) {
            ofLogWarning("This frame was not added!");
        }
    }

    // Check if the video recorder encountered any error while writing video frame or audio smaples.
    if (vidRecorder.hasVideoError()) {
        ofLogWarning("The video recorder failed to write some frames!");
    }

    if (vidRecorder.hasAudioError()) {
        ofLogWarning("The video recorder failed to write some audio samples!");
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

	// time
	dt = ofGetElapsedTimef() - t;
	t = ofGetElapsedTimef();

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

//--------------------------------------------------------------
void ofApp::draw(){
	ofClear(0,0);
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

			kinectFbo.draw(0,0);
			
			// ofEnableBlendMode(OF_BLENDMODE_ALPHA);

			// kinectFbo.getTexture().draw(0,0);

			ofEnableBlendMode(OF_BLENDMODE_ADD);
		}
	}
	if (showLogo)
	{
		logo.draw(0, 0 , drawWidth , drawHeight);
	}

	// stringstream ss;
 //    ss << "video queue size: " << vidRecorder.getVideoQueueSize() << endl
 //    << "audio queue size: " << vidRecorder.getAudioQueueSize() << endl
 //    << "FPS: " << ofGetFrameRate() << endl
 //    << (bRecording?"pause":"start") << " recording: r" << endl
 //    << (bRecording?"close current video file: c":"") << endl;

    // ofSetColor(0,0,0,100);
    // ofDrawRectangle(0, 0, 260, 75);
    // ofSetColor(255, 255, 255);
    // ofDrawBitmapString(ss.str(),15,15);

	// tmpFbo.begin();
	// ofClear(0,0,0,0);
	// ofSetColor(255);
	// ofFill();
	// ofDrawRectangle(0,0,200,200);
	// tmpFbo.end();
	// tmpFbo.draw(0,0,drawWidth,drawHeight);
	// ofSetColor(255);
	// ofDrawBitmapString("strength="+ofToString(drawForces[0].getStrength()), 10,10);
}

//--------------------------------------------------------------
void ofApp::updateConstantSources()
{	

	if (startFlow)
	{
		drawForces[0].setStrength(1 + sin(2.0*3.14*t/3));
		if (drawForces[0].getStrength()<0.5)
		{
			drawForces[0].setStrength(0);
		}

		float s = 0.1+pow(sin(2.0*3.14*t/4),16);

		drawForces[0].setStrength(s);

		float v = 1 + 0.2*sin(2.0*3.14*2/4);
		drawForces[1].setStrength(v);
	}
	else
	{
		drawForces[0].setStrength(0);
		drawForces[1].setStrength(1);
	}

	// sourceRadius = 0.1 + 0.05 * sin(2.0*3.14*t);
	// for (int i=0; i<nForces; i++)
	// {
	// 	drawForces[i].setRadius(sourceRadius);
	// }


	// tmpFbo.begin();
	// ofClear(0,0,0,0);
	// ofSetColor(255);
	// ofFill();
	// ofDrawCircle(50,50,20,20);
	// tmpFbo.end();
	
	// fluidSimulation.addDensity(tmpFbo.getTexture());

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
void ofApp::keyPressed(int key){


	
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
}

//--------------------------------------------------------------
void ofApp::audioIn(float *input, int bufferSize, int nChannels){
    if(bRecording)
        vidRecorder.addAudioSamples(input, bufferSize, nChannels);
}

//--------------------------------------------------------------
void ofApp::recordingComplete(ofxVideoRecorderOutputFileCompleteEventArgs& args){
    cout << "The recoded video file is now complete." << endl;
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
	if (key==OF_KEY_SHIFT)
	{
		threshStep = 1;
	}
	if(key=='r'){
        bRecording = !bRecording;
        if(bRecording && !vidRecorder.isInitialized()) {
            vidRecorder.setup(fileName+ofGetTimestampString()+fileExt, drawWidth, drawHeight, 30, sampleRate, channels);
//          vidRecorder.setup(fileName+ofGetTimestampString()+fileExt, vidGrabber.getWidth(), vidGrabber.getHeight(), 30); // no audio
//            vidRecorder.setup(fileName+ofGetTimestampString()+fileExt, 0,0,0, sampleRate, channels); // no video
//          vidRecorder.setupCustomOutput(vidGrabber.getWidth(), vidGrabber.getHeight(), 30, sampleRate, channels, "-vcodec mpeg4 -b 1600k -acodec mp2 -ab 128k -f mpegts udp://localhost:1234"); // for custom ffmpeg output string (streaming, etc)

            // Start recording
            vidRecorder.start();
        }
        else if(!bRecording && vidRecorder.isInitialized()) {
            vidRecorder.setPaused(true);
        }
        else if(bRecording && vidRecorder.isInitialized()) {
            vidRecorder.setPaused(false);
        }
    }
    if(key=='c'){
        bRecording = false;
        vidRecorder.close();
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
    ofRemoveListener(vidRecorder.outputFileCompleteEvent, this, &ofApp::recordingComplete);
    vidRecorder.close();
}