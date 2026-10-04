// Neon-dark instrument palette.
#pragma once

#include <QColor>
#include <QString>

namespace radar {
namespace theme {

inline QColor bg()          { return QColor(6, 10, 14); }
inline QColor panel()       { return QColor(12, 18, 24); }
inline QColor panelAlt()    { return QColor(17, 25, 33); }
inline QColor grid()        { return QColor(28, 48, 58); }
inline QColor gridBright()   { return QColor(42, 78, 92); }
inline QColor text()        { return QColor(196, 224, 232); }
inline QColor textDim()     { return QColor(104, 140, 152); }
inline QColor cyan()        { return QColor(64, 224, 255); }
inline QColor cyanDim()     { return QColor(30, 110, 130); }
inline QColor amber()       { return QColor(255, 186, 66); }
inline QColor magenta()     { return QColor(255, 92, 170); }
inline QColor green()       { return QColor(96, 255, 168); }
inline QColor red()         { return QColor(255, 86, 86); }
inline QColor violet()      { return QColor(160, 130, 255); }

// Deterministic, well-separated colour per transmitter MAC.
inline QColor forMac(quint64 hash) {
    static const QColor palette[] = {
        QColor(64, 224, 255),  QColor(96, 255, 168), QColor(255, 186, 66),
        QColor(255, 122, 122), QColor(160, 130, 255), QColor(255, 92, 170),
        QColor(120, 220, 200), QColor(200, 200, 120), QColor(140, 170, 255),
        QColor(255, 150, 90),
    };
    return palette[hash % (sizeof(palette) / sizeof(palette[0]))];
}

// Stylesheet for the whole application.
inline QString styleSheet() {
    return QString(R"(
QWidget {
    background: #060A0E;
    color: #C4E0E8;
    font-family: "DejaVu Sans", "Ubuntu", "Segoe UI", sans-serif;
    font-size: 12px;
}
QGroupBox {
    border: 1px solid #1C303A;
    border-radius: 6px;
    margin-top: 14px;
    padding: 10px 8px 8px 8px;
    background: #0C1218;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 5px;
    color: #40E0FF;
    font-weight: bold;
    letter-spacing: 1px;
}
QLabel { background: transparent; }
QLabel[role="h1"] { font-size: 17px; font-weight: bold; color: #40E0FF; letter-spacing: 3px; }
QLabel[role="h2"] { font-size: 11px; color: #688C98; letter-spacing: 2px; }
QLabel[role="metric"] { font-size: 22px; font-weight: bold; color: #C4E0E8; }
QLabel[role="metricUnit"] { font-size: 11px; color: #688C98; }
QLabel[role="warn"] { color: #FF5656; }
QLabel[role="good"] { color: #60FFA8; }
QPushButton {
    background: #111921;
    border: 1px solid #1C303A;
    border-radius: 4px;
    padding: 6px 12px;
    color: #C4E0E8;
}
QPushButton:hover { border-color: #40E0FF; color: #40E0FF; }
QPushButton:checked {
    background: #10333F;
    border-color: #40E0FF;
    color: #40E0FF;
}
QPushButton#primary {
    background: #0E3B48;
    border-color: #40E0FF;
    color: #40E0FF;
    font-weight: bold;
}
QComboBox {
    background: #111921;
    border: 1px solid #1C303A;
    border-radius: 4px;
    padding: 5px 8px;
}
QComboBox:hover { border-color: #40E0FF; }
QComboBox QAbstractItemView {
    background: #0C1218;
    border: 1px solid #1C303A;
    selection-background-color: #10333F;
    color: #C4E0E8;
}
QSlider::groove:horizontal {
    height: 4px; background: #1C303A; border-radius: 2px;
}
QSlider::handle:horizontal {
    background: #40E0FF; width: 12px; height: 12px;
    margin: -5px 0; border-radius: 6px;
}
QSpinBox, QDoubleSpinBox {
    background: #0A1016;
    border: 1px solid #1C303A;
    border-radius: 4px;
    padding: 3px 5px;
}
QCheckBox { background: transparent; spacing: 6px; }
QCheckBox::indicator {
    width: 13px; height: 13px;
    border: 1px solid #2A4E5C; border-radius: 3px; background: #0A1016;
}
QCheckBox::indicator:checked { background: #40E0FF; border-color: #40E0FF; }
QPlainTextEdit, QTextEdit {
    background: #060A0E;
    border: 1px solid #1C303A;
    border-radius: 4px;
    font-family: "DejaVu Sans Mono", "Ubuntu Mono", monospace;
    font-size: 11px;
}
QTabWidget::pane { border: 1px solid #1C303A; border-radius: 4px; top: -1px; }
QTabBar::tab {
    background: #0C1218;
    border: 1px solid #1C303A;
    padding: 6px 6px;
    margin-right: 2px;
    font-size: 11px;
    color: #688C98;
}
QTabBar::tab:selected { background: #10333F; color: #40E0FF; border-bottom-color: #40E0FF; }
QScrollBar:vertical { background: #060A0E; width: 9px; margin: 0; }
QScrollBar::handle:vertical { background: #1C303A; border-radius: 4px; min-height: 24px; }
QScrollBar::handle:vertical:hover { background: #2A4E5C; }
QScrollBar::add-line, QScrollBar::sub-line { height: 0; width: 0; }
QScrollBar:horizontal { background: #060A0E; height: 9px; margin: 0; }
QScrollBar::handle:horizontal { background: #1C303A; border-radius: 4px; min-width: 24px; }
)");
}

}  // namespace theme
}  // namespace radar