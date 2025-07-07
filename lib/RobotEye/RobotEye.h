#ifndef ROBOTEYE_H
#define ROBOTEYE_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

struct Point {
  int x;
  int y;
};

class RobotEye {
public:
  RobotEye(Adafruit_SSD1306* display);
  void setQuad(Point topLeft, Point topRight, Point bottomRight, Point bottomLeft);
  void draw();

  Point getTopLeft() const;
  Point getTopRight() const;
  Point getBottomRight() const;
  Point getBottomLeft() const;

private:
  Adafruit_SSD1306* display;
  Point tl, tr, br, bl;
};

#endif // ROBOTEYE_H