#include "testApp.h"
#include "ofAppGlutWindow.h"

int main() {
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 900, 480, OF_WINDOW);
	ofRunApp(new testApp());
}
