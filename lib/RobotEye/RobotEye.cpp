#include "RobotEye.h"

RobotEye::RobotEye(Adafruit_SSD1306* disp) {
  display = disp;
  tl = {20, 20};
  tr = {108, 20};
  br = {108, 44};
  bl = {20, 44};
}

void RobotEye::setQuad(Point topLeft, Point topRight, Point bottomRight, Point bottomLeft) {
  tl = topLeft;
  tr = topRight;
  br = bottomRight;
  bl = bottomLeft;
}

void RobotEye::draw() {
  display->clearDisplay();
  display->fillTriangle(tl.x, tl.y, tr.x, tr.y, br.x, br.y, SSD1306_WHITE);
  display->fillTriangle(tl.x, tl.y, br.x, br.y, bl.x, bl.y, SSD1306_WHITE);
  display->display();
}

Point RobotEye::getTopLeft() const { return tl; }
Point RobotEye::getTopRight() const { return tr; }
Point RobotEye::getBottomRight() const { return br; }
Point RobotEye::getBottomLeft() const { return bl; }
