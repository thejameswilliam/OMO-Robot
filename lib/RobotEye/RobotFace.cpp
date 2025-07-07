#include "RobotFace.h"
#include <Arduino.h>

RobotFace::RobotFace(RobotEye* left, RobotEye* right) {
  leftEye = left;
  rightEye = right;
}

void RobotFace::setEmotion(const String& previousEmotion, const String& emotion, float intensity) {
  intensity = constrain(intensity, 0.0, 1.0);
  String fromEmotion = previousEmotion.length() > 0 ? previousEmotion : emotion;

  auto getEmotionShape = [](const String& emo, float factor) -> std::array<Point, 4> {
    Point tl = {20, 20};
    Point tr = {108, 20};
    Point br = {108, 44};
    Point bl = {20, 44};

    if (emo == "angry") {
      tl.y -= 8 * factor;
      tr.y += 8 * factor;
    } else if (emo == "surprised") {
      tl.y -= 12 * factor;
      tr.y -= 12 * factor;
      bl.y += 12 * factor;
      br.y += 12 * factor;
    } else if (emo == "sleepy") {
      tl.y += 12 * factor;
      tr.y += 12 * factor;
      bl.y -= 12 * factor;
      br.y -= 12 * factor;
    } else if (emo == "sad") {
      tl.y += 6 * factor;
      tr.y += 6 * factor;
      bl.y += 4 * factor;
      br.y += 4 * factor;
    } else if (emo == "happy") {
      bl.y -= 6 * factor;
      br.y -= 6 * factor;
      tl.y += 2 * factor;
      tr.y += 2 * factor;
    } else if (emo == "mischievous") {
      tl.y -= 4 * factor;
      tr.y += 6 * factor;
      bl.y += 4 * factor;
      br.y -= 6 * factor;
    }

    return {tl, tr, br, bl};
  };

  auto blendPoints = [](const Point& a, const Point& b, float alpha) -> Point {
    return {
      static_cast<int>(a.x * (1.0 - alpha) + b.x * alpha),
      static_cast<int>(a.y * (1.0 - alpha) + b.y * alpha)
    };
  };

  if (intensity <= 0.0f) {
    auto shape = getEmotionShape(fromEmotion, 1.0f);
    leftEye->setQuad(shape[0], shape[1], shape[2], shape[3]);
    auto mirrorX = [](Point p) -> Point { return {128 - p.x, p.y}; };
    rightEye->setQuad(
      mirrorX(shape[0]),
      mirrorX(shape[1]),
      mirrorX(shape[2]),
      mirrorX(shape[3])
    );
    return;
  }

  if (intensity >= 1.0f) {
    auto shape = getEmotionShape(emotion, 1.0f);
    leftEye->setQuad(shape[0], shape[1], shape[2], shape[3]);
    auto mirrorX = [](Point p) -> Point { return {128 - p.x, p.y}; };
    rightEye->setQuad(
      mirrorX(shape[0]),
      mirrorX(shape[1]),
      mirrorX(shape[2]),
      mirrorX(shape[3])
    );
    return;
  }

  float eased = intensity < 0.5f ? 4 * intensity * intensity * intensity
                                 : 1 - pow(-2 * intensity + 2, 3) / 2;

  auto from = getEmotionShape(fromEmotion, 1.0f);
  auto to = getEmotionShape(emotion, 1.0f);

  Point tl = blendPoints(from[0], to[0], eased);
  Point tr = blendPoints(from[1], to[1], eased);
  Point br = blendPoints(from[2], to[2], eased);
  Point bl = blendPoints(from[3], to[3], eased);

  leftEye->setQuad(tl, tr, br, bl);

  auto mirrorX = [](Point p) -> Point { return {128 - p.x, p.y}; };
  rightEye->setQuad(
    mirrorX(tl),
    mirrorX(tr),
    mirrorX(br),
    mirrorX(bl)
  );
}

void RobotFace::draw() {
  leftEye->draw();
  rightEye->draw();
}
