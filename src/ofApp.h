#pragma once

#include "ofMain.h"
#include "ofxFlowTools.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

using namespace flowTools;

// enum modeEnum{
	
// };

class ofApp : public ofBaseApp{

	public:
		bool startFlow = true;
		bool debug = false;
		bool doReset = false;
		bool fadeOut = false;
		bool fadeIn = true;

		ofTexture tex;

		double mastBright = 1.0;

		ofxKinect kinect;
		ofxCvGrayscaleImage grayImage; // grayscale depth image
		ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
		ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
		bool bThreshWithOpenCV;
		int nearThreshold;
		int farThreshold;
		int threshStep = 1;
		bool drawDepth = true;
		ofFbo kinectFbo;

		
	    int sampleRate;
	    int channels;	    
        ofSoundStream       soundStream;

        ofPixels stagePixels;
		ofImage stageImg;
		int nframes = 0;

        void audioIn(float * input, int bufferSize, int nChannels);


	    ofFbo recordFbo;
	    ofPixels recordPixels;

		void setup();
		void update();
		void draw();
		void exit();
		void initflow();
		void initsources();
		void cycleLogo();		

		int flowWidth;
		int flowHeight;
		int drawWidth;
		int drawHeight;

		ftOpticalFlow opticalFlow;
		ftVelocityMask velocityMask;
		ftFluidSimulation fluidSimulation;
		ftParticleFlow particleFlow;
		ftVelocitySpheres velocityDots;

		// time
		float t;
		int t_ms;
		float dt;
		int t_hours;
		int t_mins;

		ofFbo tmpFbo;

		int nForces = 3;
		ftDrawForce* drawForces;

		// logo
		vector<string> images;
		int logoID;
		ofImage logo;
		bool showLogo;


		ofXml XML;
		void getXMLsources();
		string xmlStructure;
		string message;
		float sourceRadius;
		vector <float> sourceX;
		vector <float> sourceY;
		vector <float> velocityX;
		vector <float> velocityY;
		vector <float> radii;
		vector <ofFloatColor> colors;
		int sourceMode = 0;
		ofFloatColor c1;
		ofFloatColor c2;
		int NS1;


		// mouse
		// ftDrawMouseForces mouseForces;

		// Visualisations
		ofParameterGroup	visualizeParameters;
		ftDisplayScalar		displayScalar;
		ftVelocityField		velocityField;
		ftTemperatureField	temperatureField;
		ftPressureField		pressureField;
		ftVTField			velocityTemperatureField;

		// draw methods
		void updateConstantSources();
		void drawComposite() {drawComposite(0,0,drawWidth,drawHeight);}
		void drawComposite(int x, int y, int w, int  h);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
};
