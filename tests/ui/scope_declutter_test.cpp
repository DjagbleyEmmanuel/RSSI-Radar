// Scope layout test.
//
// Renders the PPI with many deliberately colliding contacts -- all at nearly the
// same range and bearing, which is the worst case the scope will meet in a room
// with several access points -- and asserts that no two drawn contact labels
// overlap.
//
// Layout only. This builds a Snapshot by hand, feeds nothing into the engine,
// and produces no reported data; it exists purely to check the scope does not
// stack blips and labels. The program's sensing path has no synthetic branch.
//
// Build and run from build/:
//
//   OBJS=$(find CMakeFiles/rssiradar.dir -name '*.o' ! -name 'main.cpp.o' \
//         ! -name '*autogen*')
//   g++ -std=c++17 -fPIC -I../include -I../ui $(pkg-config --cflags Qt6Widgets) \
//       ../tests/ui/scope_declutter_test.cpp $OBJS -o /tmp/scope_test \
//       $(pkg-config --libs Qt6Widgets Qt6Gui Qt6Core)
//   QT_QPA_PLATFORM=offscreen /tmp/scope_test
//
// Exits non-zero if any label pair overlaps.
#include <QApplication>
#include <QImage>
#include <QPainter>
#include "Widgets.hpp"
using namespace radar;
int main(int argc, char** argv){
  QApplication app(argc, argv);
  RadarScope w; w.resize(760, 700);
  Snapshot s;
  // 9 contacts clustered hard: same range, bearings 0/360/5/355 etc.
  for (int i = 0; i < 20; ++i) {
    Contact c; c.id = i + 1;
    for (int b = 0; b < 6; ++b) c.mac[b] = static_cast<uint8_t>(i * 7 + b * 3);
    c.label = "ap-" + std::to_string(i);
    c.rangeM = 2.0 + (i % 3) * 0.3;  c.rangeValid = true;
    c.bearingValid = false;
    c.levelDbm = -40.0 - i; c.presence = 1.0 - i * 0.07; c.confidence = 0.8;
    c.state = ContactState::Active; c.updates = 40;
    s.contacts.push_back(c);
  }
  w.setSnapshot(s);
  QImage img(w.size(), QImage::Format_ARGB32);
  img.fill(Qt::black);
  w.render(&img);
  img.save("/tmp/opencode/declutter.png");
  // Count overlapping label boxes. Filled in by the widget during paint, so the
  // check is made against what was actually drawn rather than what was intended.
  const int overlaps = w.labelOverlaps();
  std::printf("rendered %d contacts, %d overlapping label pairs\n",
              (int)s.contacts.size(), overlaps);
  return overlaps == 0 ? 0 : 1;
}
